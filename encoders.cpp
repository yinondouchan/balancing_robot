#include <Arduino.h>

#include "encoders.h"
#include "parameters.h"

// motor's encoder velocity in full-steps per second
float encoders_left_motor_vel;
float encoders_right_motor_vel;

// previous angle read by encoder
int16_t left_motor_prev_angle;
int16_t right_motor_prev_angle;

// read velocity from an encoder
void encoders_read_velocity(uint8_t encoder)
{
    // read the angle
    int16_t new_angle = analogRead((encoder == ENCODERS_LEFT_ENCODER) ? ENCODERS_LEFT_MOTOR_PIN : ENCODERS_RIGHT_MOTOR_PIN);

    // calculate encoder angle difference
    float prev_angle = (encoder == ENCODERS_LEFT_ENCODER) ? left_motor_prev_angle: right_motor_prev_angle;
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

    if (encoder == ENCODERS_LEFT_ENCODER)
    {
        // add a low-pass filter
        encoders_left_motor_vel = 0.5 * encoders_left_motor_vel + 0.5 * -new_motor_encoder_vel;

        // set previous angle value
        left_motor_prev_angle = new_angle;
    }
    else
    {
        // add a low-pass filter
        encoders_right_motor_vel = 0.5 * encoders_right_motor_vel + 0.5 * new_motor_encoder_vel;

        // set previous angle value
        right_motor_prev_angle = new_angle;
    }
}

// initialize encoders
void encoders_init()
{
    // set encoder pins as input pins
    pinMode(ENCODERS_LEFT_MOTOR_PIN, INPUT);
    pinMode(ENCODERS_RIGHT_MOTOR_PIN, INPUT);
  
    encoders_left_motor_vel = 0;
    left_motor_prev_angle = analogRead(ENCODERS_LEFT_MOTOR_PIN);
    
    encoders_right_motor_vel = 0;
    right_motor_prev_angle = analogRead(ENCODERS_RIGHT_MOTOR_PIN);

    // init encoder pins
    pinMode(ENCODERS_LEFT_MOTOR_PIN, INPUT);
    pinMode(ENCODERS_RIGHT_MOTOR_PIN, INPUT);
}
