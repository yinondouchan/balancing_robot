#include <Arduino.h>

extern "C"
{
  #include "avr_i2c.h"
}

#include <mpu9250.h>
#include "imu.h"
#include "bluetooth.h"
#include "config.h"

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define DIRECTION_FORWARD 0
#define DIRECTION_BACKWARD 1

#define STEP_MODE_FULL 1
#define STEP_MODE_HALF 2
#define STEP_MODE_QUARTER 4
#define STEP_MODE_EIGHTH 8
#define STEP_MODE_SIXTEENTH 16
#define STEP_MODE_THIRTY_TWOTH 32
#define STEP_MODE_AUTO 0

#define DIGITAL_POT_WRITE_CMD 0x00
#define LOOP_COUNTER_DIVISOR 39

#define DT_MICROS 4000
#define FULL_STEPS_PER_REV 200

#define LEFT_MOTOR_STEP_PIN 4
#define LEFT_MOTOR_DIR_PIN 2
#define LEFT_MOTOR_MS1_PIN 6
#define LEFT_MOTOR_MS2_PIN 5
#define LEFT_MOTOR_MS3_PIN 9
#define LEFT_MOTOR_I1_PIN 9

#define RIGHT_MOTOR_STEP_PIN 8
#define RIGHT_MOTOR_DIR_PIN 7
#define RIGHT_MOTOR_MS1_PIN 11
#define RIGHT_MOTOR_MS2_PIN 12
#define RIGHT_MOTOR_MS3_PIN 10
#define RIGHT_MOTOR_I1_PIN 10

#define ENCODER_FULL_SCALE 1024

#define STALL_SPEED_DIFF_THRESHOLD 50

#define CLOCK_SPEED_HZ 16000000

#define BALANCE_ANGLE -2.5

#define PWM_VOLTAGE 5

// IMU
MPU_9250 imu;

unsigned long timestamp;

typedef struct
{
    float angle_pid_p;
    float angle_pid_ang_vel_p;
    float vel_pid_p;
    float vel_pid_i;
    float vel_pid_d;
    float vel_lpf_tc;
    int32_t balance_vel_limit;
    float vel_pid_d2;
    float current_limit;
} parameters_t;

// remote parameter tuning tags
enum parameter_tlv_tag {
                        ANGLE_PID_P,                // inner PID loop angle proportional component
                        ANGLE_PID_ANG_VEL_P,        // inner PID loop angular velocity proportional component
                        VEL_PID_P,                  // outer PID loop velocity proportional
                        VEL_PID_I,                  // outer PID loop velocity integral
                        VEL_PID_D,                  // outer PID loop velocity derivative
                        VEL_LPF_TC,                 // velocity low-pass filter time constant [microseconds]
                        BALANCE_VEL_LIMIT,          // balancing algorithm velocity limit [full-steps per second]
                        VEL_PID_D2,
                        CURRENT_LIMIT,
                        };

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

#if MOTOR_DRIVER == MOTOR_DRIVER_A4988
const uint8_t step_modes[] = {STEP_MODE_FULL, STEP_MODE_HALF, STEP_MODE_QUARTER, STEP_MODE_EIGHTH, STEP_MODE_SIXTEENTH};
#elif MOTOR_DRIVER == MOTOR_DRIVER_MP6500
const uint8_t step_modes[] = {STEP_MODE_FULL, STEP_MODE_HALF, STEP_MODE_QUARTER, STEP_MODE_EIGHTH};
#elif MOTOR_DRIVER == MOTOR_DRIVER_TB67S249
const uint8_t step_modes[] = {STEP_MODE_FULL, STEP_MODE_HALF, STEP_MODE_QUARTER, STEP_MODE_EIGHTH, STEP_MODE_SIXTEENTH, STEP_MODE_THIRTY_TWOTH};
#endif

int32_t ctr;

float balance_point_power, prev_vel_lpf, prev_vel, bp_i;
float vel_lpf;

bool upright;
bool main_loop_triggered;

parameters_t parameters;

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

    // initialize Timer1 for main loop - a 7812.5 Hz interrupt rate
    TCCR1A = 0;    // set entire TCCR1A register to 0
    TCCR1B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    TIMSK1 |= (1 << TOIE1);

    // set fast PWM mode and set prescaler to 8
    TCCR1A += _BV(WGM10);
    TCCR1B |= _BV(CS11) | _BV(WGM12);

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

ISR(TIMER1_OVF_vect)
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
    uint8_t dir_pin = (motor == LEFT_MOTOR) ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN;
    uint8_t ms1_pin = (motor == LEFT_MOTOR) ? LEFT_MOTOR_MS1_PIN : RIGHT_MOTOR_MS1_PIN;
    uint8_t ms2_pin = (motor == LEFT_MOTOR) ? LEFT_MOTOR_MS2_PIN : RIGHT_MOTOR_MS2_PIN;
    uint8_t ms3_pin = (motor == LEFT_MOTOR) ? LEFT_MOTOR_MS3_PIN : RIGHT_MOTOR_MS3_PIN;
    
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
        }
        else if (step_mode == STEP_MODE_HALF)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_QUARTER)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
        }
        else if (step_mode == STEP_MODE_EIGHTH)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
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
        }
        else if (step_mode == STEP_MODE_HALF)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_QUARTER)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
        }
        else if (step_mode == STEP_MODE_EIGHTH)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
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

void set_current_limit(uint8_t motor, float amps)
{
    amps = constrain(amps, 0.0, 2.2);
    float analog_voltage = (2.2 - amps) / 0.63;
    uint16_t pwm_value = analog_voltage * 255 / PWM_VOLTAGE;

    analogWrite((motor == LEFT_MOTOR) ? LEFT_MOTOR_I1_PIN : RIGHT_MOTOR_I1_PIN, pwm_value);
}

void balance_point_control(int32_t desired_vel, int32_t desired_ang_vel, int32_t dt_micros)
{
    // get angle, angular velocity and velocity from IMU and encoders
    float angle = gyro_angle_x;
    float ang_vel = imu_data.gyro[0];
    int32_t vel = desired_vel - (right_motor_vel + left_motor_vel)/2;

    bp_i += parameters.vel_pid_i * vel * dt_micros;
    vel_lpf = parameters.vel_lpf_tc / (parameters.vel_lpf_tc + dt_micros) * vel_lpf + dt_micros / (parameters.vel_lpf_tc + dt_micros) * vel;
    
    int32_t angle_offset = parameters.vel_pid_p * vel + bp_i + parameters.vel_pid_d * (vel_lpf - prev_vel_lpf) / dt_micros;

    // angle control
    float power_gain = parameters.angle_pid_ang_vel_p * ang_vel + parameters.angle_pid_p * (angle - BALANCE_ANGLE - angle_offset);// - 0.005 * vel; // - (50.0  / dt_micros) * (vel - prev_vel);
    // add to power the tilt-angle element and the I element of the speed
    balance_point_power += power_gain * dt_micros / 1000;
    balance_point_power = constrain(balance_point_power, -parameters.balance_vel_limit, parameters.balance_vel_limit);

    float power = balance_point_power + (parameters.vel_pid_d2  / dt_micros) * (vel_lpf - prev_vel_lpf);
    power = constrain(power, -parameters.balance_vel_limit, parameters.balance_vel_limit); 
    
    // control motor velocities
    set_velocity(LEFT_MOTOR, power + desired_ang_vel, STEP_MODE_AUTO);
    set_velocity(RIGHT_MOTOR, power - desired_ang_vel, STEP_MODE_AUTO);

    prev_vel = vel;
    prev_vel_lpf = vel_lpf;
}

void reset_motors()
{
    bp_i = 0;
    vel_lpf = 0;

    prev_vel = 0;
    prev_vel_lpf = 0;
    //prev_angle = 0;
    balance_point_power = 0;

    right_motor_vel = 0;
    left_motor_vel = 0;
}

void set_default_parameters()
{
    parameters.angle_pid_p = 0.15;
    parameters.angle_pid_ang_vel_p = 0.08;
    parameters.vel_pid_p = 0.010;
    parameters.vel_pid_i = 0.00000002;
    parameters.vel_pid_d = 11000.0;
    parameters.vel_lpf_tc = 300000.0;
    parameters.balance_vel_limit = 6000;
    parameters.vel_pid_d2 = 200000.0;
}

void set_parameter(uint8_t tag, uint8_t len, uint8_t *value)
{
    // remote parameter tuning tags
    switch(tag)
    {
        case ANGLE_PID_P:
        parameters.angle_pid_p = *(float*)value;
        break;
        case ANGLE_PID_ANG_VEL_P:
        parameters.angle_pid_ang_vel_p = *(float*)value;
        break;
        case VEL_PID_P:
        parameters.vel_pid_p = *(float*)value;
        break;
        case VEL_PID_I:
        parameters.vel_pid_i = *(float*)value;
        break;
        case VEL_PID_D:
        parameters.vel_pid_d = *(float*)value;
        break;
        case VEL_LPF_TC:
        parameters.vel_lpf_tc = *(float*)value;
        break;
        case BALANCE_VEL_LIMIT:
        parameters.balance_vel_limit = *(int32_t*)value;
        break;
        case VEL_PID_D2:
        parameters.vel_pid_d2 = *(float*)value;
        break;
        case CURRENT_LIMIT:
        set_current_limit(LEFT_MOTOR, *(float*)value);
        set_current_limit(RIGHT_MOTOR, *(float*)value);
        break;
    }
}

void setup()
{
    set_default_parameters();
    setup_timers();
    
    Serial.begin(9600);

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

    set_current_limit(LEFT_MOTOR, 1.5);
    set_current_limit(RIGHT_MOTOR, 1.5);

    // init imu
    imu_init(&imu);

    // calibrate gyro
    Serial.println("Calibrating gyro");
    imu_calibrate_gyro(&imu);
    Serial.println("Done calibrating gyro");

    // init complementary filter
    compl_filter_init();

    // init bluetooth
    bt_init();
    bt_set_param_callback(set_parameter);

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
    prev_vel_lpf = 0;
    bp_i = 0;
    vel_lpf;

    ctr = 0;

    main_loop_triggered = false;

    // is the robot upright?
    upright = true;
}

void loop()
{
    // 250Hz loop rate
    if (!main_loop_triggered && (loop_counter == 0))
    {
      
        //set_current_limit(LEFT_MOTOR, 2);
        //set_current_limit(RIGHT_MOTOR, 2);
        //set_velocity(LEFT_MOTOR, min(ctr, 2000), STEP_MODE_AUTO);
        //ctr++;
        main_loop_triggered = true;
        
        // read accel and gyro values
        read_accel_and_gyro(&imu);
        compl_filter_read(DT_MICROS);
        
        bt_read();
        
        if ((cf_angle_x < -60) || (cf_angle_x > 60)) upright = false;
        if (!upright)
        {
            // control motor velocities
            set_velocity(LEFT_MOTOR, 0, STEP_MODE_QUARTER);
            set_velocity(RIGHT_MOTOR, 0, STEP_MODE_QUARTER);
            upright = (cf_angle_x > ((int32_t)BALANCE_ANGLE - 10)) && (cf_angle_x < ((int32_t)BALANCE_ANGLE + 10));
            if (upright) gyro_angle_x = cf_angle_x;
            reset_motors();
            return;
        }

        // filter angular velocity control to avoid those annoying discrete level sounds
        bt_desired_vel_diff_lpf = 0.1 * bt_desired_vel_diff + 0.9 * bt_desired_vel_diff_lpf;
        balance_point_control(bt_desired_vel, -bt_desired_vel_diff_lpf, DT_MICROS);
    }
    else if (loop_counter != 0)
    {
        main_loop_triggered = false;
    }
}
