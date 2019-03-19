#include <Arduino.h>

extern "C"
{
  #include "avr_i2c.h"
}

#include <mpu9250.h>
#include "imu.h"
#include "bluetooth.h"

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define DIRECTION_FORWARD 0
#define DIRECTION_BACKWARD 1

#define STEP_MODE_FULL 1
#define STEP_MODE_HALF 2
#define STEP_MODE_QUARTER 4
#define STEP_MODE_EIGHTH 8
#define STEP_MODE_SIXTEENTH 16
#define STEP_MODE_AUTO 0

#define DIGITAL_POT_WRITE_CMD 0x00
#define LOOP_COUNTER_DIVISOR 4

#define DT_MICROS 4000
#define FULL_STEPS_PER_REV 200

#define LEFT_MOTOR_STEP_PIN 4
#define LEFT_MOTOR_DIR_PIN 2
#define LEFT_MOTOR_MS1_PIN 6
#define LEFT_MOTOR_MS2_PIN 5
#define LEFT_MOTOR_MS3_PIN 3

#define RIGHT_MOTOR_STEP_PIN 8
#define RIGHT_MOTOR_DIR_PIN 7
#define RIGHT_MOTOR_MS1_PIN 11
#define RIGHT_MOTOR_MS2_PIN 12
#define RIGHT_MOTOR_MS3_PIN 10

#define ENCODER_FULL_SCALE 1024

#define STALL_SPEED_DIFF_THRESHOLD 50

#define CLOCK_SPEED_HZ 16000000

#define BALANCE_ANGLE -2.5

// IMU
MPU_9250 imu;

unsigned long timestamp;

// timer counter for left and right motors
volatile int32_t left_motor_counter;
volatile int32_t right_motor_counter;

// timer for main loop
volatile int32_t loop_counter;

// timer ticks per motor step
int32_t left_motor_nticks;
int32_t right_motor_nticks;

// motor's step velocity in full-steps per second
float left_motor_vel;
float right_motor_vel;

// motor's encoder velocity in full-steps per second
float left_motor_encoder_vel;
float right_motor_encoder_vel;

// previous angle read by encoder
int16_t left_motor_prev_angle;
int16_t right_motor_prev_angle;

const uint8_t step_modes[] = {STEP_MODE_FULL, STEP_MODE_HALF, STEP_MODE_QUARTER, STEP_MODE_EIGHTH, STEP_MODE_SIXTEENTH};

int32_t ctr;

float balance_point_power, prev_vel, bp_i;
float vel_lpf;

bool upright;
bool main_loop_triggered;

void setup_timers()
{
    cli();

    // initialize Timer0 for controlling right stepper motor
    TCCR0A = 0;    // set entire TCCR0A register to 0
    TCCR0B = 0;    // set entire TCCR0B register to 0 
                   // (as we do not know the initial  values)

    TIMSK0 |= (1 << OCIE0A);

    TCCR0B |= _BV(CS01);
    OCR0A = 255;
    TCCR0A |= _BV(WGM01);

    // initialize Timer1 for main loop
    TCCR1A = 0;    // set entire TCCR1A register to 0
    TCCR1B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    TIMSK1 |= (1 << OCIE1A);

    // set CTC mode and set prescaler to 8
    TCCR1B |= _BV(CS11) | _BV(WGM12) | _BV(WGM13);
    ICR1 = 1999;

    // initialize Timer2 for controlling left stepper motor
    TCCR2A = 0;    // set entire TCCR2A register to 0
    TCCR2B = 0;    // set entire TCCR2B register to 0 
                   // (as we do not know the initial  values)

    TIMSK2 |= (1 << OCIE2A);
    TCCR2B |= _BV(CS21);
    OCR2A = 255;
    TCCR2A |= _BV(WGM21);
    
    // enable global interrupts:
    sei();
}

ISR(TIMER1_COMPA_vect)
{
    loop_counter = (loop_counter >= LOOP_COUNTER_DIVISOR) ? 0 : loop_counter + 1;
}

ISR(TIMER2_COMPA_vect)
{
    // called every compare-match
    digitalWrite(LEFT_MOTOR_STEP_PIN, left_motor_counter >= (left_motor_nticks - 1));
    left_motor_counter = (left_motor_counter >= (left_motor_nticks - 1)) ? 0 : left_motor_counter + 1;
}

ISR(TIMER0_COMPA_vect)
{
    digitalWrite(RIGHT_MOTOR_STEP_PIN, right_motor_counter >= (right_motor_nticks - 1));
    right_motor_counter = (right_motor_counter >= (right_motor_nticks - 1)) ? 0 : right_motor_counter + 1;
}

void read_encoder_velocity()
{
    int16_t new_angle = analogRead(A0);

    // calculate encoder angle difference
    int16_t left_encoder_angle_diff = (new_angle - left_motor_prev_angle);

    // deal with angle overflow, i.e. transition from ENCODER_FULL_SCALE - 1 to 0 or backwards
    if (abs(left_encoder_angle_diff) > ENCODER_FULL_SCALE/2)
    {
        // angle_diff should be positive
        if (new_angle < left_motor_prev_angle) left_encoder_angle_diff += ENCODER_FULL_SCALE;
        // angle_diff should be negative
        else left_encoder_angle_diff -= ENCODER_FULL_SCALE;
    }

    // find velocity in encoder step per second
    float new_motor_encoder_vel = 1000000.0 * (float)left_encoder_angle_diff / DT_MICROS;
    // convert to full-steps per second
    new_motor_encoder_vel = new_motor_encoder_vel * FULL_STEPS_PER_REV / ENCODER_FULL_SCALE;

    // add a low-pass filter
    left_motor_encoder_vel = 0.9 * left_motor_encoder_vel + 0.1 * new_motor_encoder_vel;
    
    left_motor_prev_angle = new_angle;
}

uint8_t get_auto_step_mode(float velocity)
{
    uint8_t num_of_step_modes = sizeof(step_modes)/sizeof(step_modes[0]);

    for (int i = (num_of_step_modes - 1); i >= 0; i--)
    {
        int32_t step_mode_nticks_value = CLOCK_SPEED_HZ / (velocity * 8 * step_modes[i] * 128);
        if (step_mode_nticks_value >= 2) return step_modes[i];
    }

    return STEP_MODE_FULL;
}

// set the velocity of a motor with a specific step_mode (full step or microstepping)
void set_velocity(uint8_t motor, float velocity, uint8_t step_mode)
{
    if (motor == LEFT_MOTOR)
    {
        // write direction
        digitalWrite(LEFT_MOTOR_DIR_PIN, velocity < 0 ? DIRECTION_BACKWARD: DIRECTION_FORWARD);
        left_motor_vel = velocity;
        velocity = abs(velocity);

        if (step_mode == STEP_MODE_AUTO) step_mode = get_auto_step_mode(velocity);
        
        // write stepping mode
        if (step_mode == STEP_MODE_FULL)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_HALF)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_QUARTER)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_EIGHTH)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_SIXTEENTH)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, HIGH);
        }
        
        set_step_rate_left(velocity * step_mode);
    }
    if (motor == RIGHT_MOTOR)
    {
        // write direction
        digitalWrite(RIGHT_MOTOR_DIR_PIN, velocity < 0 ? DIRECTION_BACKWARD: DIRECTION_FORWARD);
        right_motor_vel = velocity;
        velocity = abs(velocity);

        if (step_mode == STEP_MODE_AUTO) step_mode = get_auto_step_mode(velocity);
        
        // write stepping mode
        if (step_mode == STEP_MODE_FULL)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_HALF)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_QUARTER)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_EIGHTH)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_SIXTEENTH)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, HIGH);
        }
        
        set_step_rate_right(velocity * step_mode);
    }
}

void set_step_rate_left(float velocity)
{
    if (velocity == 0)
    {
        OCR0A = 255;
        left_motor_nticks = 1;
        return;
    }
    // determine highest nticks such that OCR can be lower than 256
    int32_t max_n_ticks = max(ceil((float)CLOCK_SPEED_HZ / (256 * 8 * velocity)), 2);
    left_motor_nticks = max_n_ticks;
    OCR2A = CLOCK_SPEED_HZ / (max_n_ticks * 8 * velocity) - 1;
}

void set_step_rate_right(float velocity)
{

    if (velocity == 0)
    {
        OCR0A = 255;
        right_motor_nticks = 1;
        return;
    }
    
    // determine highest nticks such that OCR can be lower than 256
    int32_t max_n_ticks = max(ceil((float)CLOCK_SPEED_HZ / (256 * 8 * velocity)), 2);
    right_motor_nticks = max_n_ticks;
    OCR0A = CLOCK_SPEED_HZ / (max_n_ticks * 8 * velocity) - 1;
}

void balance_point_control(int32_t desired_vel, int32_t desired_ang_vel, int32_t dt_micros)
{
    // get angle, angular velocity and velocity from IMU and encoders
    float angle = cf_angle_x;
    float ang_vel = imu_data.gyro[0];
    int32_t vel = desired_vel - (right_motor_vel + left_motor_vel)/2;

    bp_i += 0.00000015 * vel * dt_micros;
    vel_lpf = 300000.0 / (300000.0 + dt_micros) * vel_lpf + dt_micros / (300000.0 + dt_micros) * vel;
    
    int32_t angle_offset = 0.02 * vel + bp_i + 30000.0 * (vel_lpf - prev_vel) / dt_micros;

    // angle control
    float power_gain = 0.08 * ang_vel + 0.15 * (angle - BALANCE_ANGLE - angle_offset);// - 0.005 * vel; // - (50.0  / dt_micros) * (vel - prev_vel);
    // add to power the tilt-angle element and the I element of the speed
    balance_point_power += power_gain * dt_micros / 1000;
    balance_point_power = constrain(balance_point_power, -3000, 3000);

    float power = balance_point_power;// + (200000.0  / dt_micros) * (vel_lpf - prev_vel);
    
    // control motor velocities
    set_velocity(LEFT_MOTOR, power + desired_ang_vel, STEP_MODE_AUTO);
    set_velocity(RIGHT_MOTOR, power - desired_ang_vel, STEP_MODE_AUTO);

    prev_vel = vel_lpf;
}

void reset_motors()
{
    bp_i = 0;
    vel_lpf = 0;
    
    prev_vel = 0;
    //prev_angle = 0;
    balance_point_power = 0;

    right_motor_vel = 0;
    left_motor_vel = 0;
}

void setup()
{
    setup_timers();
    
    Serial.begin(115200);

    // set up STEP and DIR pins
    pinMode(LEFT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_MS2_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_MS2_PIN, OUTPUT);
    
    digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);
    digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_STEP_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);

    // init imu
    imu_init(&imu);
    compl_filter_init();

    // init bluetooth
    bt_init();

    // encoder
    pinMode(A0, INPUT);

    loop_counter = 0;

    // set up left motor
    left_motor_nticks = 0;
    left_motor_counter = 0;
    left_motor_vel = 0;
    left_motor_encoder_vel = 0;
    left_motor_prev_angle = analogRead(A0);

    // set up right motor
    right_motor_nticks = 0;
    right_motor_counter = 0;
    right_motor_vel = 0;
    right_motor_encoder_vel = 0;

    // balancing algorithm
    balance_point_power = 0;
    prev_vel = 0;
    bp_i = 0;
    vel_lpf;

    ctr = 0;

    main_loop_triggered = false;

    // is the robot upright?
    upright = true;

    // calibrate gyro
    Serial.println("Calibrating gyro");
    imu_calibrate_gyro(&imu);
    Serial.println("Done calibrating gyro");
}

void loop()
{
    // 250Hz loop rate
    if (!main_loop_triggered && (loop_counter == 0))
    {
        main_loop_triggered = true;
        // read accel and gyro values
        //set_velocity(LEFT_MOTOR, ctr, STEP_MODE_AUTO);
        //set_velocity(RIGHT_MOTOR, 10000, STEP_MODE_FULL);
        read_accel_and_gyro(&imu);
        compl_filter_read(DT_MICROS);
    
        if ((cf_angle_x < -60) || (cf_angle_x > 60)) upright = false;
        if (!upright)
        {
            // control motor velocities
            set_velocity(LEFT_MOTOR, 0, STEP_MODE_QUARTER);
            set_velocity(RIGHT_MOTOR, 0, STEP_MODE_QUARTER);
            upright = (cf_angle_x > ((int32_t)BALANCE_ANGLE - 10)) && (cf_angle_x < ((int32_t)BALANCE_ANGLE + 10));
            reset_motors();
            while(loop_counter < LOOP_COUNTER_DIVISOR);
            return;
        }

        bt_read_joystick_control();
        balance_point_control(bt_desired_vel, -bt_desired_vel_diff, DT_MICROS); 
    }
    else if (loop_counter != 0)
    {
        main_loop_triggered = false;
    }
}
