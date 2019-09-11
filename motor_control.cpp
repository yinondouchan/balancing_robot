#include <Arduino.h>

#include "motor_control.h"
#include "encoders.h"
#include "parameters.h"

// timer counter for left and right motors
volatile int32_t left_motor_counter;
volatile int32_t right_motor_counter;

// number of timer compare match interrupts per STEP pulse sent to left motor driver
int32_t left_motor_nticks;

// number of timer compare match interrupts per STEP pulse sent to right motor driver
int32_t right_motor_nticks;

// motor's step velocity in full-steps per second
float motor_control_left_motor_vel;
float motor_control_right_motor_vel;

// true if one of the robot's motors are stalled
bool motor_control_on_stall;

// step modes for various stepper motor drivers
#if MC_MOTOR_DRIVER == MOTOR_DRIVER_A4988
const uint8_t step_modes[] = {MC_STEP_MODE_FULL, MC_STEP_MODE_HALF, MC_STEP_MODE_QUARTER, MC_STEP_MODE_EIGHTH, MC_STEP_MODE_SIXTEENTH};
#elif MC_MOTOR_DRIVER == MOTOR_DRIVER_MP6500
const uint8_t step_modes[] = {MC_STEP_MODE_FULL, MC_STEP_MODE_HALF, MC_STEP_MODE_QUARTER, MC_STEP_MODE_EIGHTH};
#elif MC_MOTOR_DRIVER == MOTOR_DRIVER_TB67S249
const uint8_t step_modes[] = {MC_STEP_MODE_FULL, MC_STEP_MODE_HALF, MC_STEP_MODE_QUARTER, MC_STEP_MODE_EIGHTH, MC_STEP_MODE_SIXTEENTH, MC_STEP_MODE_THIRTY_TWOTH};
#endif

// compare match interrupt for controlling left motor
ISR(TIMER2_COMPA_vect)
{
    // called every compare-match and controls pulse frequency
    digitalWrite(MC_LEFT_MOTOR_STEP_PIN, left_motor_counter >= (left_motor_nticks - 1));

    // send pulse every nticks interrupts
    left_motor_counter = (left_motor_counter >= (left_motor_nticks - 1)) ? 0 : left_motor_counter + 1;
}

// compare match interrupt for controlling right motor
ISR(TIMER0_COMPA_vect)
{
    // called every compare-match and controls pulse frequency
    digitalWrite(MC_RIGHT_MOTOR_STEP_PIN, right_motor_counter >= (right_motor_nticks - 1));

    // send pulse every nticks interrupts
    right_motor_counter = (right_motor_counter >= (right_motor_nticks - 1)) ? 0 : right_motor_counter + 1;
}

// initialize motor control module
void motor_control_init()
{
    // set up STEP and DIR pins for left motor
    pinMode(MC_LEFT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(MC_LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(MC_LEFT_MOTOR_MS2_PIN, OUTPUT);

    // set up STEP and DIR pins for right motor
    pinMode(MC_RIGHT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(MC_RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(MC_RIGHT_MOTOR_MS2_PIN, OUTPUT);
  
    // set up left motor
    left_motor_nticks = 0;
    left_motor_counter = 0;
    motor_control_left_motor_vel = 0;

     // set up right motor
    right_motor_nticks = 0;
    right_motor_counter = 0;
    motor_control_right_motor_vel = 0;

    // stall detection
    motor_control_on_stall = false;

    // start with motors disabled.
    motor_control_disable_motors();
}

// set pulse rate for left motor driver
void motor_control_set_step_rate_left(float velocity)
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
void motor_control_set_step_rate_right(float velocity)
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

// set the velocity of a motor with a specific step_mode (full step or microstepping)
void motor_control_set_velocity(uint8_t motor, float velocity, uint8_t step_mode)
{
    // determine the output pins for the right driver
    uint8_t dir_pin = (motor == MC_LEFT_MOTOR) ? MC_LEFT_MOTOR_DIR_PIN : MC_RIGHT_MOTOR_DIR_PIN;
    uint8_t ms1_pin = (motor == MC_LEFT_MOTOR) ? MC_LEFT_MOTOR_MS1_PIN : MC_RIGHT_MOTOR_MS1_PIN;
    uint8_t ms2_pin = (motor == MC_LEFT_MOTOR) ? MC_LEFT_MOTOR_MS2_PIN : MC_RIGHT_MOTOR_MS2_PIN;
    uint8_t ms3_pin = (motor == MC_LEFT_MOTOR) ? MC_LEFT_MOTOR_MS3_PIN : MC_RIGHT_MOTOR_MS3_PIN;
    
    if (motor == MC_LEFT_MOTOR)
    {
        // write direction
        digitalWrite(MC_LEFT_MOTOR_DIR_PIN, velocity < 0 ? MC_MOTOR_DIRECTION_BACKWARD: MC_MOTOR_DIRECTION_FORWARD);

        // assume actual motor velocity equals to desired velocity
        motor_control_left_motor_vel = velocity;
        velocity = abs(velocity);

        // determine step mode
        if (step_mode == MC_STEP_MODE_AUTO) step_mode = motor_control_get_auto_step_mode(velocity);
        
        // write stepping mode for left motor
        if (step_mode == MC_STEP_MODE_FULL)
        {
            digitalWrite(MC_LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(MC_LEFT_MOTOR_MS2_PIN, LOW);
            digitalWrite(MC_LEFT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == MC_STEP_MODE_HALF)
        {
            digitalWrite(MC_LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(MC_LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(MC_LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == MC_STEP_MODE_QUARTER)
        {
            digitalWrite(MC_LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(MC_LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(MC_LEFT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == MC_STEP_MODE_EIGHTH)
        {
            digitalWrite(MC_LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(MC_LEFT_MOTOR_MS2_PIN, LOW);
            digitalWrite(MC_LEFT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == MC_STEP_MODE_SIXTEENTH)
        {
            digitalWrite(MC_LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(MC_LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(MC_LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == MC_STEP_MODE_THIRTY_TWOTH)
        {
            digitalWrite(MC_LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(MC_LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(MC_LEFT_MOTOR_MS3_PIN, HIGH);
        }

        // control step rate of left motor (microsteps per second)
        motor_control_set_step_rate_left(velocity * step_mode);
    }
    if (motor == MC_RIGHT_MOTOR)
    {
        // write direction
        digitalWrite(MC_RIGHT_MOTOR_DIR_PIN, velocity < 0 ? MC_MOTOR_DIRECTION_BACKWARD: MC_MOTOR_DIRECTION_FORWARD);
        motor_control_right_motor_vel = velocity;
        velocity = abs(velocity);

        // determine step mode
        if (step_mode == MC_STEP_MODE_AUTO) step_mode = motor_control_get_auto_step_mode(velocity);
        
        // write stepping mode
        if (step_mode == MC_STEP_MODE_FULL)
        {
            digitalWrite(MC_RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(MC_RIGHT_MOTOR_MS2_PIN, LOW);
            digitalWrite(MC_RIGHT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == MC_STEP_MODE_HALF)
        {
            digitalWrite(MC_RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(MC_RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(MC_RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == MC_STEP_MODE_QUARTER)
        {
            digitalWrite(MC_RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(MC_RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(MC_RIGHT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == MC_STEP_MODE_EIGHTH)
        {
            digitalWrite(MC_RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(MC_RIGHT_MOTOR_MS2_PIN, LOW);
            digitalWrite(MC_RIGHT_MOTOR_MS3_PIN, HIGH);
        }
        else if (step_mode == MC_STEP_MODE_SIXTEENTH)
        {
            digitalWrite(MC_RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(MC_RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(MC_RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == MC_STEP_MODE_THIRTY_TWOTH)
        {
            digitalWrite(MC_RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(MC_RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(MC_RIGHT_MOTOR_MS3_PIN, HIGH);
        }

        // control step rate of right motor (microsteps per second)
        motor_control_set_step_rate_right(velocity * step_mode);
    }
}

// automatic microstepping mode selection as a function of velocity
uint8_t motor_control_get_auto_step_mode(float velocity)
{
    // choose the finest microstepping mode where it will be possible to reach this velocity without a too high timer interrupt rate
    uint8_t num_of_step_modes = sizeof(step_modes)/sizeof(step_modes[0]);
    for (int i = (num_of_step_modes - 1); i >= 0; i--)
    {
        int32_t step_mode_nticks_value = CLOCK_SPEED_HZ / (velocity * MC_STEP_TIMER_CLOCK_PRESCALER * step_modes[i] * 128);
        if (step_mode_nticks_value >= 2) return step_modes[i];
    }

    return MC_STEP_MODE_FULL;
}

// reset motor control
void motor_control_reset()
{
    motor_control_right_motor_vel = 0;
    motor_control_left_motor_vel = 0;
}

// distable the motors
void motor_control_disable_motors()
{
    // causes the motors to coast rather than brake
    digitalWrite(MC_LEFT_MOTOR_MS1_PIN, LOW);
    digitalWrite(MC_LEFT_MOTOR_MS2_PIN, LOW);
    digitalWrite(MC_LEFT_MOTOR_MS3_PIN, LOW);
    digitalWrite(MC_RIGHT_MOTOR_MS1_PIN, LOW);
    digitalWrite(MC_RIGHT_MOTOR_MS2_PIN, LOW);
    digitalWrite(MC_RIGHT_MOTOR_MS3_PIN, LOW);
}

// control velocity with a stall detection layer built upon it
void set_velocity_with_stall_detection(int8_t motor, float velocity, uint8_t step_mode)
{
    // velocity difference in full steps between control velocity and encoder velocity
    float actual_vel_error = (motor == MC_LEFT_MOTOR) ? (motor_control_left_motor_vel - encoders_left_motor_vel) : (motor_control_right_motor_vel - encoders_right_motor_vel);

    // true if stall detected (velocity difference is higher than the defined threshold)
    motor_control_on_stall = abs(actual_vel_error) >= MC_STALL_SPEED_DIFF_THRESHOLD;

    if (motor_control_on_stall)
    {        
        // stall occured - immediately set control velocity to what the encoder read
        if (motor == MC_LEFT_MOTOR) motor_control_left_motor_vel = encoders_left_motor_vel;
        else motor_control_right_motor_vel = encoders_right_motor_vel;
    }
    else // no stall
    {
        // calculate difference between desired velocity and actual velocity
        float desired_vel_diff = velocity - ((motor == MC_LEFT_MOTOR) ? motor_control_left_motor_vel : motor_control_right_motor_vel);

        // sign of the above difference
        int sign = (desired_vel_diff > 0) - (desired_vel_diff < 0);

        if (abs(desired_vel_diff) <= MC_STALL_ACCEL_SPEED)
        {
            // when velocity difference is small enough equalize between desired difference and actual control difference
            if (motor == MC_LEFT_MOTOR) motor_control_left_motor_vel = velocity;
            else motor_control_right_motor_vel = velocity;
        }
        else
        {
            // when velocity difference is still big accelerate towards desired velocity
            if (motor == MC_LEFT_MOTOR) motor_control_left_motor_vel += MC_STALL_ACCEL_SPEED * sign;
            else motor_control_right_motor_vel += MC_STALL_ACCEL_SPEED * sign;
        }

        // set the velocity accordingly
        motor_control_set_velocity(motor, motor == (MC_LEFT_MOTOR) ? motor_control_left_motor_vel : motor_control_right_motor_vel, step_mode);
    }
}
