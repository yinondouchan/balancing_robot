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

// current hardware prescaler for right and left motor
uint16_t right_motor_hw_prescaler;
uint16_t left_motor_hw_prescaler;

// motor's step velocity in full-steps per second
float motor_control_left_motor_vel;
float motor_control_right_motor_vel;

// true if one of the robot's motors are stalled
bool motor_control_on_stall;

// true of motors are enabled
bool motor_control_motors_enabled;

// how many consecutive rocks left/right motor is stalling
uint16_t left_motor_stall_count;
uint16_t right_motor_stall_count;

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
    pinMode(MC_LEFT_MOTOR_NENABLE_PIN, OUTPUT);

    // set up STEP and DIR pins for right motor
    pinMode(MC_RIGHT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(MC_RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(MC_RIGHT_MOTOR_MS2_PIN, OUTPUT);
    pinMode(MC_RIGHT_MOTOR_NENABLE_PIN, OUTPUT);
  
    // set up left motor
    left_motor_nticks = 0;
    left_motor_counter = 0;
    motor_control_left_motor_vel = 0;
    left_motor_hw_prescaler = 8;

     // set up right motor
    right_motor_nticks = 0;
    right_motor_counter = 0;
    motor_control_right_motor_vel = 0;
    right_motor_hw_prescaler = 8;

    // stall detection
    motor_control_on_stall = false;

    // start with motors disabled.
    motor_control_disable_motors();

    left_motor_stall_count = 0;
    right_motor_stall_count = 0;
}

// set pulse rate for a motor driver
void motor_control_set_step_rate(int8_t motor, float velocity)
{
    if (velocity == 0)
    {
        // if on standstill set timer so it will run on lowest interrupt rate
        if (motor == MC_LEFT_MOTOR)
        {
            OCR2A = 255;
            left_motor_nticks = 1;
        }
        else // motor == MC_RIGHT_MOTOR
        {
            OCR0A = 255;
            right_motor_nticks = 1;
        }
        
        return;
    }
    
    // determine highest nticks (timer interrupts per step pulse) such that OCR can be of the highest value, yet lower than 256
    uint16_t hardware_prescaler = 8;// motor_control_set_highest_hw_prescaler(motor, combined_prescaler);
    int32_t software_prescaler = max(ceil((float)CLOCK_SPEED_HZ / (256 * hardware_prescaler * velocity)), 2);
    //int32_t software_prescaler = max(combined_prescaler / hardware_prescaler, 1) + 1;

    // given the nticks calculate the OCR value
    if (motor == MC_LEFT_MOTOR)
    {
        left_motor_nticks = software_prescaler;
        OCR2A = CLOCK_SPEED_HZ / (software_prescaler * hardware_prescaler * velocity) - 1;
    }
    else // motor == MC_RIGHT_MOTOR
    {
        right_motor_nticks = software_prescaler;
        OCR0A = CLOCK_SPEED_HZ / (software_prescaler * hardware_prescaler * velocity) - 1;
    }
}

// set highest possible hardware prescaler given a required combined prescaler for maintaining a specific pulse rate
uint16_t motor_control_set_highest_hw_prescaler(int8_t motor, uint16_t combined_prescaler)
{   
    if (combined_prescaler >= 1024)
    {
        // do nothing if prescaler is already set
        if ((motor == MC_LEFT_MOTOR) && (left_motor_hw_prescaler == 1024)) return 1024;
        else if ((motor == MC_RIGHT_MOTOR) && (right_motor_hw_prescaler == 1024)) return 1024;
        
        // set hardware prescaler to 1024
        if (motor == MC_LEFT_MOTOR)
        {
            left_motor_hw_prescaler = 1024;
            TCCR2B = TCCR2B & 0b11111000 | 0b11111111;
        }
        else
        {
            right_motor_hw_prescaler = 1024;
            TCCR0B = TCCR0B & 0b11111000 | 0b11111101;
        }

        return 1024;
    }
    else if (combined_prescaler >= 256)
    {   
        // do nothing if prescaler is already set
        if ((motor == MC_LEFT_MOTOR) && (left_motor_hw_prescaler == 256)) return 256;
        else if ((motor == MC_RIGHT_MOTOR) && (right_motor_hw_prescaler == 256)) return 256;
      
        // set hardware prescaler to 256
        if (motor == MC_LEFT_MOTOR)
        {
            left_motor_hw_prescaler = 256;
            TCCR2B = TCCR2B & 0b11111000 | 0b11111110;
        }
        else
        {
            right_motor_hw_prescaler = 256;
            TCCR0B = TCCR0B & 0b11111000 | 0b11111100;
        }

        return 256;
    }
    else if (combined_prescaler >= 64)
    {
        // do nothing if prescaler is already set
        if ((motor == MC_LEFT_MOTOR) && (left_motor_hw_prescaler == 64)) return 64;
        else if ((motor == MC_RIGHT_MOTOR) && (right_motor_hw_prescaler == 64)) return 64;
      
        // set hardware prescaler to 64
        if (motor == MC_LEFT_MOTOR)
        {
            left_motor_hw_prescaler = 64;
            TCCR2B = TCCR2B & 0b11111000 | 0b11111100;
        }
        else
        {
            right_motor_hw_prescaler = 64;
            TCCR0B = TCCR0B & 0b11111000 | 0b11111011;
        }

        return 64;
    }
    else
    {
        // do nothing if prescaler is already set
        if ((motor == MC_LEFT_MOTOR) && (left_motor_hw_prescaler == 8)) return 8;
        else if ((motor == MC_RIGHT_MOTOR) && (right_motor_hw_prescaler == 8)) return 8;
      
        // set hardware prescaler to 8
        if (motor == MC_LEFT_MOTOR)
        {
            left_motor_hw_prescaler = 8;
            TCCR2B = TCCR2B & 0b11111000 | 0b11111010;
        }
        else
        {
            right_motor_hw_prescaler = 8;
            TCCR0B = TCCR0B & 0b11111000 | 0b11111010;
        }

        return 8;
    }
    /*else // combined_prescaler < 8. That's pretty darn fast. Are you chasing an airplane?
    {
        // do nothing if prescaler is already set
        if ((motor == MC_LEFT_MOTOR) && (left_motor_hw_prescaler == 1)) return 1;
        else if ((motor == MC_RIGHT_MOTOR) && (right_motor_hw_prescaler == 1)) return 1;
    
        // set hardware prescaler to 1
        if (motor == MC_LEFT_MOTOR)
        {
            left_motor_hw_prescaler = 1;
            TCCR2B = TCCR2B & 0b11111000 | 0b11111001;
        }
        else
        {
            right_motor_hw_prescaler = 8;
            TCCR0B = TCCR0B & 0b11111000 | 0b11111001;
        }

        return 1;
    }*/
}

// set the velocity of a motor with a specific step_mode (full step or microstepping)
void motor_control_set_velocity(uint8_t motor, float velocity, uint8_t step_mode)
{
    // determine the output pins for the right driver
    uint8_t dir_pin = (motor == MC_LEFT_MOTOR) ? MC_LEFT_MOTOR_DIR_PIN : MC_RIGHT_MOTOR_DIR_PIN;
    uint8_t ms1_pin = (motor == MC_LEFT_MOTOR) ? MC_LEFT_MOTOR_MS1_PIN : MC_RIGHT_MOTOR_MS1_PIN;
    uint8_t ms2_pin = (motor == MC_LEFT_MOTOR) ? MC_LEFT_MOTOR_MS2_PIN : MC_RIGHT_MOTOR_MS2_PIN;
    uint8_t ms3_pin = (motor == MC_LEFT_MOTOR) ? MC_LEFT_MOTOR_MS3_PIN : MC_RIGHT_MOTOR_MS3_PIN;
    
    // write direction
    digitalWrite(dir_pin, velocity < 0 ? MC_MOTOR_DIRECTION_BACKWARD: MC_MOTOR_DIRECTION_FORWARD);

    // assume actual motor velocity equals to desired velocity
    if (motor == MC_LEFT_MOTOR) motor_control_left_motor_vel = velocity;
    else motor_control_right_motor_vel = velocity;
    velocity = abs(velocity);

    // determine step mode
    if (step_mode == MC_STEP_MODE_AUTO) step_mode = motor_control_get_auto_step_mode(velocity);
#if MOTOR_DRIVER == MOTOR_DRIVER_TB67S249
    // write stepping mode for left motor
    if (step_mode == MC_STEP_MODE_FULL)
    {
        digitalWrite(ms1_pin, LOW);
        digitalWrite(ms2_pin, LOW);
        digitalWrite(ms3_pin, HIGH);
    }
    else if (step_mode == MC_STEP_MODE_HALF)
    {
        digitalWrite(ms1_pin, LOW);
        digitalWrite(ms2_pin, HIGH);
        digitalWrite(ms3_pin, LOW);
    }
    else if (step_mode == MC_STEP_MODE_QUARTER)
    {
        digitalWrite(ms1_pin, LOW);
        digitalWrite(ms2_pin, HIGH);
        digitalWrite(ms3_pin, HIGH);
    }
    else if (step_mode == MC_STEP_MODE_EIGHTH)
    {
        digitalWrite(ms1_pin, HIGH);
        digitalWrite(ms2_pin, LOW);
        digitalWrite(ms3_pin, HIGH);
    }
    else if (step_mode == MC_STEP_MODE_SIXTEENTH)
    {
        digitalWrite(ms1_pin, HIGH);
        digitalWrite(ms2_pin, HIGH);
        digitalWrite(ms3_pin, LOW);
    }
    else if (step_mode == MC_STEP_MODE_THIRTY_TWOTH)
    {
        digitalWrite(ms1_pin, HIGH);
        digitalWrite(ms2_pin, HIGH);
        digitalWrite(ms3_pin, HIGH);
    }
#elif MOTOR_DRIVER == MOTOR_DRIVER_DRV8825
    // write stepping mode for left motor
    if (step_mode == MC_STEP_MODE_FULL)
    {
        digitalWrite(ms1_pin, LOW);
        digitalWrite(ms2_pin, LOW);
        digitalWrite(ms3_pin, LOW);
    }
    else if (step_mode == MC_STEP_MODE_HALF)
    {
        digitalWrite(ms1_pin, HIGH);
        digitalWrite(ms2_pin, LOW);
        digitalWrite(ms3_pin, LOW);
    }
    else if (step_mode == MC_STEP_MODE_QUARTER)
    {
        digitalWrite(ms1_pin, LOW);
        digitalWrite(ms2_pin, HIGH);
        digitalWrite(ms3_pin, LOW);
    }
    else if (step_mode == MC_STEP_MODE_EIGHTH)
    {
        digitalWrite(ms1_pin, HIGH);
        digitalWrite(ms2_pin, HIGH);
        digitalWrite(ms3_pin, LOW);
    }
    else if (step_mode == MC_STEP_MODE_SIXTEENTH)
    {
        digitalWrite(ms1_pin, LOW);
        digitalWrite(ms2_pin, LOW);
        digitalWrite(ms3_pin, HIGH);
    }
    else if (step_mode == MC_STEP_MODE_THIRTY_TWOTH)
    {
        digitalWrite(ms1_pin, HIGH);
        digitalWrite(ms2_pin, LOW);
        digitalWrite(ms3_pin, HIGH);
    }
#endif
    // control step rate of left motor (microsteps per second)
    motor_control_set_step_rate(motor, velocity * step_mode);
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
    motor_control_reset();
    motor_control_set_step_rate(MC_LEFT_MOTOR, 0);
    motor_control_set_step_rate(MC_RIGHT_MOTOR, 0);
  
    // causes the motors to coast rather than brake
    /*digitalWrite(MC_LEFT_MOTOR_MS1_PIN, LOW);
    digitalWrite(MC_LEFT_MOTOR_MS2_PIN, LOW);
    digitalWrite(MC_LEFT_MOTOR_MS3_PIN, LOW);
    digitalWrite(MC_RIGHT_MOTOR_MS1_PIN, LOW);
    digitalWrite(MC_RIGHT_MOTOR_MS2_PIN, LOW);
    digitalWrite(MC_RIGHT_MOTOR_MS3_PIN, LOW);*/
    digitalWrite(MC_LEFT_MOTOR_NENABLE_PIN, HIGH);
    digitalWrite(MC_RIGHT_MOTOR_NENABLE_PIN, HIGH);
    motor_control_motors_enabled = false;
}

// distable the motors
void motor_control_enable_motors()
{
    // nothing to do if motors are already enabled
    if (motor_control_motors_enabled) return;
    
    digitalWrite(MC_LEFT_MOTOR_NENABLE_PIN, LOW);
    digitalWrite(MC_RIGHT_MOTOR_NENABLE_PIN, LOW);
    motor_control_motors_enabled = true;
}

// control velocity with a stall detection layer built upon it
void motor_control_set_velocity_with_stall_detection(int8_t motor, float velocity, uint8_t step_mode)
{
    // velocity difference in full steps between control velocity and encoder velocity
    float actual_vel_error = (motor == MC_LEFT_MOTOR) ? (motor_control_left_motor_vel - encoders_left_motor_vel) : (motor_control_right_motor_vel - encoders_right_motor_vel);

    // true if stall detected (velocity difference is higher than the defined threshold)
    bool vel_discrepancy = abs(actual_vel_error) >= MC_STALL_SPEED_DIFF_THRESHOLD;
    /*if (vel_discrepancy)
    {
        if (motor == MC_LEFT_MOTOR) left_motor_stall_count++;
        else right_motor_stall_count++;
    }
    else
    {
        if (motor == MC_LEFT_MOTOR) left_motor_stall_count = 0;
        else right_motor_stall_count = 0;
    }

    // check whether to declare that stall detected
    uint16_t stall_count = motor == MC_LEFT_MOTOR ? left_motor_stall_count : right_motor_stall_count;*/
    motor_control_on_stall = vel_discrepancy;// && (stall_count >= MC_STALL_DETECTION_TRIGGER_COUNT);

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
