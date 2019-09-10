/*
 * Main file for the balancing robot.
 */

#include <Arduino.h>

extern "C"
{
  #include "avr_i2c.h"
}

#include <mpu9250.h>
#include "imu.h"
#include "serial_comm.h"
#include "config.h"

// motor index
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

// motor direction
#define DIRECTION_FORWARD 0
#define DIRECTION_BACKWARD 1

// microstepping modes
#define STEP_MODE_FULL 1
#define STEP_MODE_HALF 2
#define STEP_MODE_QUARTER 4
#define STEP_MODE_EIGHTH 8
#define STEP_MODE_SIXTEENTH 16
#define STEP_MODE_THIRTY_TWOTH 32
#define STEP_MODE_AUTO 0    // automatic microstepping selection

// how many timer interrups to count until main loop is triggered
#define LOOP_COUNTER_DIVISOR 40

// main loop time interval in microseconds
#define DT_MICROS 5000

// main loop time interval in seconds
#define DT_SECONDS 0.005

// clock prescaler for the timers responsible for the stepper motors (timer 0 and timer 2)
#define STEP_TIMER_CLOCK_PRESCALER 8

// number of full steps per motor revolution
#define FULL_STEPS_PER_REV 200

// left motor step and direction pins
#define LEFT_MOTOR_STEP_PIN 4
#define LEFT_MOTOR_DIR_PIN 2

// left motor microstepping pins. Actual microstepping values depend on the motor driver used
#define LEFT_MOTOR_MS1_PIN 6
#define LEFT_MOTOR_MS2_PIN 5
#define LEFT_MOTOR_MS3_PIN 9

// left motor encoder pin
#define LEFT_MOTOR_ENCODER_PIN A1

// right motor step and direction pins
#define RIGHT_MOTOR_STEP_PIN 8
#define RIGHT_MOTOR_DIR_PIN 7

// right motor microstepping pins. Actual microstepping values depend on the motor driver used
#define RIGHT_MOTOR_MS1_PIN 11
#define RIGHT_MOTOR_MS2_PIN 12
#define RIGHT_MOTOR_MS3_PIN 100

// right motor encoder pin
#define RIGHT_MOTOR_ENCODER_PIN A0

// number of encoder values per full revolution of the motor
#define ENCODER_FULL_SCALE 1024

// velocity difference threshold for triggering the stall detection and handling mechanism
// threshold is in units of full-steps/sec between desired velocity and what the encoder reads
#define STALL_SPEED_DIFF_THRESHOLD 200

// the amount of acceleration back to desired velocity given a stall
#define STALL_ACCEL_SPEED 100

// clock speed
#define CLOCK_SPEED_HZ 16000000

// the angle in degrees read from the IMU where it is assumed that the robot is perfectly balanced
#define BALANCE_ANGLE -2.5

// a nice macro for calculating the sign of a number
#define SIGN(x) (((x) > 0) - ((x) < 0))

// maximum acceleration and deceleration in full steps per second
#define MAX_ACCEL 2000
#define MAX_DECCEL 2000

// the IMU
MPU_9250 imu;

// tunable parameters
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

// tunable parameters tags
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

// number of timer compare match interrupts per STEP pulse sent to left motor driver
int32_t left_motor_nticks;

// number of timer compare match interrupts per STEP pulse sent to right motor driver
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

// step modes for various stepper motor drivers
#if MOTOR_DRIVER == MOTOR_DRIVER_A4988
const uint8_t step_modes[] = {STEP_MODE_FULL, STEP_MODE_HALF, STEP_MODE_QUARTER, STEP_MODE_EIGHTH, STEP_MODE_SIXTEENTH};
#elif MOTOR_DRIVER == MOTOR_DRIVER_MP6500
const uint8_t step_modes[] = {STEP_MODE_FULL, STEP_MODE_HALF, STEP_MODE_QUARTER, STEP_MODE_EIGHTH};
#elif MOTOR_DRIVER == MOTOR_DRIVER_TB67S249
const uint8_t step_modes[] = {STEP_MODE_FULL, STEP_MODE_HALF, STEP_MODE_QUARTER, STEP_MODE_EIGHTH, STEP_MODE_SIXTEENTH, STEP_MODE_THIRTY_TWOTH};
#endif

// output velocity to motor from balancing algorithm
float balance_point_power;

// filtered and unfiltered velocity error from previous cycle
float prev_vel_error_lpf;
float prev_vel;

// I component of the balancing algorithm's top layer
float bp_i;

// low pass filtered velocity error
float vel_error_lpf;

// true if the robot is upright
bool upright;

// true if the robot is on emergency stop 
bool on_estop;

// true if main loop was already triggered for an iteration
bool main_loop_triggered;

// control velocity after applying acceleration and deceleration constraints
float serial_comm_desired_vel_constrained;

// true if one of the robot's motors are stalled
bool on_stall;

// tunable parameters
parameters_t parameters;

// setup the robot's timers
// timer 0 is used for controlling right stepper motor
// timer 1 is used for controlling main loop
// timer 2 is used for controlling left stepper motor
void setup_timers()
{
    // clear all interrupts
    cli();

    // initialize Timer0 for controlling right stepper motor
    TCCR0A = 0;    // set entire TCCR0A register to 0
    TCCR0B = 0;    // set entire TCCR0B register to 0 
                   // (as we do not know the initial  values)

    // enable output compare interrupt for timer 0
    TIMSK0 |= (1 << OCIE0A);

    // set initial output compare register value for timer 0
    OCR0A = 255;

    // set CTC mode (clear timer on compare-match) and set clock prescaler to 8
    TCCR0A |= _BV(WGM01);
    TCCR0B |= _BV(CS01);

    // initialize Timer1 for main loop - a 7812.5 Hz interrupt rate
    TCCR1A = 0;    // set entire TCCR1A register to 0
    TCCR1B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    // enable overflow interrupt for timer 1
    TIMSK1 |= (1 << TOIE1);

    // set fast PWM mode and set prescaler to 8
    TCCR1A += _BV(WGM10);
    TCCR1B |= _BV(CS11) | _BV(WGM12);

    // initialize Timer2 for controlling left stepper motor
    TCCR2A = 0;    // set entire TCCR2A register to 0
    TCCR2B = 0;    // set entire TCCR2B register to 0 
                   // (as we do not know the initial  values)

    // set initial output compare register value for timer 2
    OCR2A = 255;

    // enable output compare interrupt for timer 0
    TIMSK2 |= (1 << OCIE2A);

    // CTC mode and clock prescaler 8 (same as timer 0)
    TCCR2B |= _BV(CS21);
    TCCR2A |= _BV(WGM21);
    
    // enable global interrupts:
    sei();
}

// overflow interrupt for timer 1 (main loop timer)
ISR(TIMER1_OVF_vect)
{
    // count until loop counter reaches the desired value to trigger the main loop
    loop_counter = (loop_counter >= (LOOP_COUNTER_DIVISOR - 1)) ? 0 : loop_counter + 1;
}

ISR(TIMER2_COMPA_vect)
{
    // called every compare-match and controls pulse frequency
    digitalWrite(LEFT_MOTOR_STEP_PIN, left_motor_counter >= (left_motor_nticks - 1));

    // send pulse every nticks interrupts
    left_motor_counter = (left_motor_counter >= (left_motor_nticks - 1)) ? 0 : left_motor_counter + 1;
}

ISR(TIMER0_COMPA_vect)
{
    // called every compare-match and controls pulse frequency
    digitalWrite(RIGHT_MOTOR_STEP_PIN, right_motor_counter >= (right_motor_nticks - 1));

    // send pulse every nticks interrupts
    right_motor_counter = (right_motor_counter >= (right_motor_nticks - 1)) ? 0 : right_motor_counter + 1;
}

// automatic microstepping mode selection as a function of velocity
uint8_t get_auto_step_mode(float velocity)
{
    // choose the finest microstepping mode where it will be possible to reach this velocity without a too high timer interrupt rate
    uint8_t num_of_step_modes = sizeof(step_modes)/sizeof(step_modes[0]);
    for (int i = (num_of_step_modes - 1); i >= 0; i--)
    {
        int32_t step_mode_nticks_value = CLOCK_SPEED_HZ / (velocity * STEP_TIMER_CLOCK_PRESCALER * step_modes[i] * 128);
        if (step_mode_nticks_value >= 2) return step_modes[i];
    }

    return STEP_MODE_FULL;
}

// set the velocity of a motor with a specific step_mode (full step or microstepping)
void set_velocity(uint8_t motor, float velocity, uint8_t step_mode)
{
    // determine the output pins for the right driver
    uint8_t dir_pin = (motor == LEFT_MOTOR) ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN;
    uint8_t ms1_pin = (motor == LEFT_MOTOR) ? LEFT_MOTOR_MS1_PIN : RIGHT_MOTOR_MS1_PIN;
    uint8_t ms2_pin = (motor == LEFT_MOTOR) ? LEFT_MOTOR_MS2_PIN : RIGHT_MOTOR_MS2_PIN;
    uint8_t ms3_pin = (motor == LEFT_MOTOR) ? LEFT_MOTOR_MS3_PIN : RIGHT_MOTOR_MS3_PIN;
    
    if (motor == LEFT_MOTOR)
    {
        // write direction
        digitalWrite(LEFT_MOTOR_DIR_PIN, velocity < 0 ? DIRECTION_BACKWARD: DIRECTION_FORWARD);

        // assume actual motor velocity equals to desired velocity
        left_motor_vel = velocity;
        velocity = abs(velocity);

        // determine step mode
        if (step_mode == STEP_MODE_AUTO) step_mode = get_auto_step_mode(velocity);
        
        // write stepping mode for left motor
        if (step_mode == STEP_MODE_FULL)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == STEP_MODE_HALF)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_QUARTER)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == STEP_MODE_EIGHTH)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == STEP_MODE_SIXTEENTH)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_THIRTY_TWOTH)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, HIGH);
        }

        // control step rate of left motor (microsteps per second)
        set_step_rate_left(velocity * step_mode);
    }
    if (motor == RIGHT_MOTOR)
    {
        // write direction
        digitalWrite(RIGHT_MOTOR_DIR_PIN, velocity < 0 ? DIRECTION_BACKWARD: DIRECTION_FORWARD);
        right_motor_vel = velocity;
        velocity = abs(velocity);

        // determine step mode
        if (step_mode == STEP_MODE_AUTO) step_mode = get_auto_step_mode(velocity);
        
        // write stepping mode
        if (step_mode == STEP_MODE_FULL)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == STEP_MODE_HALF)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_QUARTER)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == STEP_MODE_EIGHTH)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == STEP_MODE_SIXTEENTH)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_THIRTY_TWOTH)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, HIGH);
        }

        // control step rate of right motor (microsteps per second)
        set_step_rate_right(velocity * step_mode);
    }
}

// control velocity with a stall detection layer built upon it
void set_velocity_with_stall_detection(int8_t motor, float velocity, uint8_t step_mode)
{
    // velocity difference in full steps between control velocity and encoder velocity
    float actual_vel_error = (motor == LEFT_MOTOR) ? (left_motor_vel - left_motor_encoder_vel) : (right_motor_vel - right_motor_encoder_vel);

    // true if stall detected (velocity difference is higher than the defined threshold)
    on_stall = abs(actual_vel_error) >= STALL_SPEED_DIFF_THRESHOLD;

    if (on_stall)
    {        
        // stall occured - immediately set control velocity to what the encoder read
        if (motor == LEFT_MOTOR) left_motor_vel = left_motor_encoder_vel;
        else right_motor_vel = right_motor_encoder_vel;
    }
    else // no stall
    {
        // calculate difference between desired velocity and actual velocity
        float desired_vel_diff = velocity - ((motor == LEFT_MOTOR) ? left_motor_vel : right_motor_vel);

        // sign of the above difference
        int sign = (desired_vel_diff > 0) - (desired_vel_diff < 0);

        if (abs(desired_vel_diff) <= STALL_ACCEL_SPEED)
        {
            // when velocity difference is small enough equalize between desired difference and actual control difference
            if (motor == LEFT_MOTOR) left_motor_vel = velocity;
            else right_motor_vel = velocity;
        }
        else
        {
            // when velocity difference is still big accelerate towards desired velocity
            if (motor == LEFT_MOTOR) left_motor_vel += STALL_ACCEL_SPEED * sign;
            else right_motor_vel += STALL_ACCEL_SPEED * sign;
        }

        // set the velocity accordingly
        set_velocity(motor, motor == (LEFT_MOTOR) ? left_motor_vel : right_motor_vel, step_mode);
    }
}

// read velocity from an encoder
void read_encoder_velocity(uint8_t motor)
{
    // read the angle
    int16_t new_angle = analogRead((motor == LEFT_MOTOR) ? LEFT_MOTOR_ENCODER_PIN : RIGHT_MOTOR_ENCODER_PIN);

    // calculate encoder angle difference
    float prev_angle = (motor == LEFT_MOTOR) ? left_motor_prev_angle: right_motor_prev_angle;
    int16_t encoder_angle_diff = (new_angle - prev_angle);

    // deal with angle overflow, i.e. transition from ENCODER_FULL_SCALE - 1 to 0 or backwards
    if (abs(encoder_angle_diff) > ENCODER_FULL_SCALE/2)
    {
        // angle_diff should be positive
        if (new_angle < prev_angle) encoder_angle_diff += ENCODER_FULL_SCALE;
        // angle_diff should be negative
        else encoder_angle_diff -= ENCODER_FULL_SCALE;
    }

    // find velocity in encoder steps per second
    float new_motor_encoder_vel = -1000000.0 * (float)encoder_angle_diff / DT_MICROS;
    
    // convert to full-steps per second
    new_motor_encoder_vel = new_motor_encoder_vel * FULL_STEPS_PER_REV / ENCODER_FULL_SCALE;

    if (motor == LEFT_MOTOR)
    {
        // add a low-pass filter
        left_motor_encoder_vel = 0.5 * left_motor_encoder_vel + 0.5 * -new_motor_encoder_vel;

        // set previous angle value
        left_motor_prev_angle = new_angle;
    }
    else
    {
        // add a low-pass filter
        right_motor_encoder_vel = 0.5 * right_motor_encoder_vel + 0.5 * new_motor_encoder_vel;

        // set previous angle value
        right_motor_prev_angle = new_angle;
    }
}

// set pulse rate for left motor driver
void set_step_rate_left(float velocity)
{
    if (velocity == 0)
    {
        // if on standstill set timer so it will run on lowest interrupt rate
        OCR0A = 255;
        left_motor_nticks = 1;
        return;
    }
    
    // determine highest nticks (timer interrupts per step pulse) such that OCR can be of the highest value, yet lower than 256
    int32_t max_n_ticks = max(ceil((float)CLOCK_SPEED_HZ / (256 * 8 * velocity)), 2);
    left_motor_nticks = max_n_ticks;

    // given the nticks calculate the OCR value
    OCR2A = CLOCK_SPEED_HZ / (max_n_ticks * 8 * velocity) - 1;
}

// set pulse rate for right motor driver
void set_step_rate_right(float velocity)
{

    if (velocity == 0)
    {
        // if on standstill set timer so it will run on lowest interrupt rate
        OCR0A = 255;
        right_motor_nticks = 1;
        return;
    }
    
    // determine highest nticks (timer interrupts per step pulse) such that OCR can be of the highest value, yet lower than 256
    int32_t max_n_ticks = max(ceil((float)CLOCK_SPEED_HZ / (256 * 8 * velocity)), 2);
    right_motor_nticks = max_n_ticks;

    // given the nticks calculate the OCR value
    OCR0A = CLOCK_SPEED_HZ / (max_n_ticks * 8 * velocity) - 1;
}

/* 
 *  the balancing algorithm - consists of two layers of control and some additional tweaks
 *  
 *  bottom layer: controls motor velocity as a function of the tilt angle and tilt angular velocity.
 *  It differs from a PID controller by the output being an integral of a linear combination of the tilt angle and the tilt angular velocity.
 *  
 *  top layer: controls tilt angle as a function of the desired velocity.
 *  This layer is implemented as a PID controller with the control output being the tilt angle 
 *  and the error signal being the error between the desired motor velocity and the actual motor velocity
 *  
 *  additional tweaks:
 *  - an additional D element on the velocity error is added to the output of the aforementioned two layer controller. 
 *  This nearly abolishes wobbliness of the robot at higher speeds
 *  - top layer: change I component only when control tells the robot to stop (desired vel and turn rate are 0). 
 *  This removes unnecessary wobbliness in lower speeds by decreasing I component windup.
 */
void balance_control(int32_t desired_vel, int32_t desired_turn_rate, int32_t dt_micros)
{
    // tilt angle angle
    float angle = cf_angle_x;

    // tilt angular velocity
    float ang_vel = imu_data.gyro[0];

    // velocity error (desired velocity - actual velocity)
    float vel_error;

    if (on_stall)
    {
        // if detected a stall count on velocity read from the encoder rather than the velocity sent to the drivers
        vel_error = desired_vel - (right_motor_encoder_vel + left_motor_encoder_vel)/2;

        // bias balancing more towards encoder velocity to keep its output velocity close to encoder velocity - increases stability in case of a stall
        balance_point_power = 0.9 * balance_point_power + 0.1 * (right_motor_encoder_vel + left_motor_encoder_vel)/2;
    }
    else
    {
        // no stall - rely on velocity sent to the drivers since it is not noisy as the encoders
        vel_error = desired_vel - (right_motor_vel + left_motor_vel)/2;
    }

    if ((desired_vel == 0) && (desired_turn_rate == 0))
    {
        // only change I component when in stop in order to compensate for balance angle error
        bp_i += parameters.vel_pid_i * vel_error * dt_micros;
    }

    // apply low pass filter on velocity error
    vel_error_lpf = parameters.vel_lpf_tc / (parameters.vel_lpf_tc + dt_micros) * vel_error_lpf + dt_micros / (parameters.vel_lpf_tc + dt_micros) * vel_error;

    // top control layer - PID controller which controls tilt angle as a function of velocity error
    int32_t angle_offset = parameters.vel_pid_p * vel_error + bp_i + parameters.vel_pid_d * (vel_error_lpf - prev_vel_error_lpf) / dt_micros;

    // bottom control layer - controls motor velocity as a function of the desired tilt angle
    float power_gain = parameters.angle_pid_ang_vel_p * ang_vel + parameters.angle_pid_p * (angle - BALANCE_ANGLE - angle_offset);
    
    // integrate the output from the bottom control layer and constrain it
    balance_point_power += power_gain * dt_micros / 1000;
    balance_point_power = constrain(balance_point_power, -parameters.balance_vel_limit, parameters.balance_vel_limit);

    // add and constrain a D term as a function of velocity error to smoothen out wobbliness in higher speeds of the robot
    float power = balance_point_power + (parameters.vel_pid_d2  / dt_micros) * (vel_error_lpf - prev_vel_error_lpf);
    power = constrain(power, -parameters.balance_vel_limit, parameters.balance_vel_limit); 
    
    // control motor velocities with the obtained output and desired turn rate
    set_velocity_with_stall_detection(LEFT_MOTOR, power + desired_turn_rate, STEP_MODE_AUTO);
    set_velocity_with_stall_detection(RIGHT_MOTOR, power - desired_turn_rate, STEP_MODE_AUTO);

    // store filtered and unfiltered velocity error values for next iteration
    prev_vel = vel_error;
    prev_vel_error_lpf = vel_error_lpf;
}

// reset the motors
void reset_motors()
{
    bp_i = 0;
    vel_error_lpf = 0;

    prev_vel = 0;
    prev_vel_error_lpf = 0;
    balance_point_power = 0;

    right_motor_vel = 0;
    left_motor_vel = 0;
}

// set default tunable parameters
void set_default_parameters()
{
    // bottom layer control tilt angle proportional
    parameters.angle_pid_p = 0.12;

    // bottom layer control tilt angular velocity proportional
    parameters.angle_pid_ang_vel_p = 0.06;

    // top layer control velocity proportional
    parameters.vel_pid_p = 0.012;

    // top layer control velocity integral
    parameters.vel_pid_i = 0.000000025;

    // top layer control velocity derivative
    parameters.vel_pid_d = 15000.0;

    // time constant in millisecond for velocity low pass filter
    parameters.vel_lpf_tc = 300000.0;

    // output velocity limit in full-steps/sec
    parameters.balance_vel_limit = 6000;

    // the additional derivative term in the balancing control algorithm
    parameters.vel_pid_d2 = 200000.0;
}

// set a tunable parameter
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
    }
}

// distable the motors
void disable_motors()
{
    // causes the motors to coast rather than brake
    digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
    digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
    digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
}

// toggle emergency stop
void toggle_estop()
{
    on_estop = !on_estop;
}

// apply acceleration and deceleration constraints to control velocity
void calculate_constrained_control_velocity()
{
    // calculate difference between unconstrained and constraint control velocity
    float vel_diff = serial_comm_desired_vel - serial_comm_desired_vel_constrained;

    // calculate the sign of the above
    int8_t vel_sign = SIGN(vel_diff);

    // check if should accelerate or decelerate
    bool accel = abs(serial_comm_desired_vel) > abs(serial_comm_desired_vel_constrained);

    // set acceleration/decceleration value
    float vel_derivative = accel ? vel_sign * MAX_ACCEL * DT_SECONDS : vel_sign * MAX_DECCEL * DT_SECONDS;

    // if control velocity violates acceleration/deceleration constrains, constrain it. If not - output the same as the control velocity.
    if (abs(vel_derivative) < abs(vel_diff)) serial_comm_desired_vel_constrained += vel_derivative;
    else serial_comm_desired_vel_constrained = serial_comm_desired_vel;
}

// setup
void setup()
{
    // set the default tunable parameters
    set_default_parameters();

    // set up the timers
    setup_timers();

    // set up serial
    Serial.begin(9600);

    // set up STEP and DIR pins for left motor
    pinMode(LEFT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_MS2_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_ENCODER_PIN, INPUT);

    // set up STEP and DIR pins for right motor
    pinMode(RIGHT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_MS2_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_ENCODER_PIN, INPUT);

    // start with motors disabled.
    disable_motors();
    
    // init imu
    imu_init(&imu);

    // calibrate gyro
    Serial.println("Calibrating gyro");
    imu_calibrate_gyro(&imu);
    Serial.println("Done calibrating gyro");

    // init complementary filter
    compl_filter_init();

    // init bluetooth
    serial_comm_init();
    serial_comm_set_param_callback(set_parameter);
    serial_comm_set_estop_callback(toggle_estop);

    // init encoder pins
    pinMode(LEFT_MOTOR_ENCODER_PIN, INPUT);
    pinMode(RIGHT_MOTOR_ENCODER_PIN, INPUT);

    loop_counter = 0;

    // set up left motor
    left_motor_nticks = 0;
    left_motor_counter = 0;
    left_motor_vel = 0;
    left_motor_encoder_vel = 0;
    left_motor_prev_angle = analogRead(LEFT_MOTOR_ENCODER_PIN);

    // set up right motor
    right_motor_nticks = 0;
    right_motor_counter = 0;
    right_motor_vel = 0;
    right_motor_encoder_vel = 0;
    right_motor_prev_angle = analogRead(RIGHT_MOTOR_ENCODER_PIN);

    // set up balancing algorithm variables
    balance_point_power = 0;
    prev_vel_error_lpf = 0;
    bp_i = 0;
    vel_error_lpf;

    // emergency stop
    on_estop = false;

    // stall detection
    on_stall = false;

    // control velocity after applying acceleration and deceleration constraints
    serial_comm_desired_vel_constrained = 0;

    // prevents main loop being triggered twice per timer interrupt
    main_loop_triggered = false;

    // is the robot upright?
    upright = true;
}

void loop()
{
    // 200Hz loop rate
    if (!main_loop_triggered && (loop_counter == 0))
    {
        main_loop_triggered = true;
        
        // read accel and gyro values
        read_accel_and_gyro(&imu);

        // read tilt angle
        compl_filter_read(DT_MICROS);

        // read encoders
        read_encoder_velocity(LEFT_MOTOR);
        read_encoder_velocity(RIGHT_MOTOR);

        // read data from serial
        serial_comm_read();

        // detect when robot is not upright
        if ((cf_angle_x < -55) || (cf_angle_x > 55)) upright = false;
        if (!upright || on_estop)
        {
            // robot is either not upright or emergency stop is on
            
            // disable the motors
            disable_motors();

            // check if robot is upright again
            upright = (cf_angle_x > ((int32_t)BALANCE_ANGLE - 10)) && (cf_angle_x < ((int32_t)BALANCE_ANGLE + 10));

            if (upright) gyro_angle_x = cf_angle_x;

            // reset desired velocity
            serial_comm_desired_vel_constrained = 0;

            // reset motor control variables
            reset_motors();
            return;
        }

        calculate_constrained_control_velocity();

        // filter angular velocity control to avoid those annoying discrete level sounds
        serial_comm_desired_vel_diff_lpf = 0.1 * serial_comm_desired_vel_diff + 0.9 * serial_comm_desired_vel_diff_lpf;

        // main balancing algorithm
        balance_control(serial_comm_desired_vel_constrained, - serial_comm_desired_vel_diff_lpf, DT_MICROS);
    }
    else if (loop_counter != 0)
    {
        main_loop_triggered = false;
    }
}
