#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// left motor step and direction pins
#define MC_LEFT_MOTOR_STEP_PIN 4
#define MC_LEFT_MOTOR_DIR_PIN 2

// right motor step and direction pins
#define MC_RIGHT_MOTOR_STEP_PIN 8
#define MC_RIGHT_MOTOR_DIR_PIN 7

// motor index
#define MC_LEFT_MOTOR 0
#define MC_RIGHT_MOTOR 1

// left motor microstepping pins. Actual microstepping values depend on the motor driver used
#define MC_LEFT_MOTOR_MS1_PIN 6
#define MC_LEFT_MOTOR_MS2_PIN 5
#define MC_LEFT_MOTOR_MS3_PIN 9

// right motor microstepping pins. Actual microstepping values depend on the motor driver used
#define MC_RIGHT_MOTOR_MS1_PIN 11
#define MC_RIGHT_MOTOR_MS2_PIN 12
#define MC_RIGHT_MOTOR_MS3_PIN 100

// motor direction
#define MC_MOTOR_DIRECTION_FORWARD 0
#define MC_MOTOR_DIRECTION_BACKWARD 1

// microstepping modes
#define MC_STEP_MODE_FULL 1
#define MC_STEP_MODE_HALF 2
#define MC_STEP_MODE_QUARTER 4
#define MC_STEP_MODE_EIGHTH 8
#define MC_STEP_MODE_SIXTEENTH 16
#define MC_STEP_MODE_THIRTY_TWOTH 32
#define MC_STEP_MODE_AUTO 0    // automatic microstepping selection

// clock prescaler for the timers responsible for the stepper motors (timer 0 and timer 2)
#define MC_STEP_TIMER_CLOCK_PRESCALER 8

// velocity difference threshold for triggering the stall detection and handling mechanism
// threshold is in units of full-steps/sec between desired velocity and what the encoder reads
#define MC_STALL_SPEED_DIFF_THRESHOLD 200

// the amount of acceleration back to desired velocity given a stall
#define MC_STALL_ACCEL_SPEED 100

extern float motor_control_left_motor_vel;
extern float motor_control_right_motor_vel;

extern bool motor_control_on_stall;

// initialize motor control module
void motor_control_init();

// set pulse rate for left motor driver
void motor_control_set_step_rate_left(float velocity);

// set pulse rate for right motor driver
void motor_control_set_step_rate_right(float velocity);

// set the velocity of a motor with a specific step_mode (full step or microstepping)
void motor_control_set_velocity(uint8_t motor, float velocity, uint8_t step_mode);

// automatic microstepping mode selection as a function of velocity
uint8_t motor_control_get_auto_step_mode(float velocity);

// reset motor control
void motor_control_reset();

// distable the motors
void motor_control_disable_motors();

// control velocity with a stall detection layer built upon it
void set_velocity_with_stall_detection(int8_t motor, float velocity, uint8_t step_mode);

#endif // MOTOR_CONTROL_H
