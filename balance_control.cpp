#include <Arduino.h>

#include "balance_control.h"
#include "imu.h"
#include "motor_control.h"
#include "encoders.h"
#include "parameters.h"

// output velocity to motor from balancing algorithm
float balance_point_power;

// filtered and unfiltered velocity error from previous cycle
float prev_vel_error_lpf;
float prev_vel;

// I component of the balancing algorithm's top layer
float bp_i;

// low pass filtered velocity error
float vel_error_lpf;

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
    float angle = imu_cf_angle_x;

    // tilt angular velocity
    float ang_vel = imu_data.gyro[0];

    // velocity error (desired velocity - actual velocity)
    float vel_error;

    if (motor_control_on_stall)
    {
        // if detected a stall count on velocity read from the encoder rather than the velocity sent to the drivers
        vel_error = desired_vel - (encoders_right_motor_vel + encoders_left_motor_vel)/2;

        // bias balancing more towards encoder velocity to keep its output velocity close to encoder velocity - increases stability in case of a stall
        balance_point_power = 0.9 * balance_point_power + 0.1 * (encoders_right_motor_vel + encoders_left_motor_vel)/2;
    }
    else
    {
        // no stall - rely on velocity sent to the drivers since it is not noisy as the encoders
        vel_error = desired_vel - (motor_control_right_motor_vel + motor_control_left_motor_vel)/2;
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
    set_velocity_with_stall_detection(MC_LEFT_MOTOR, power + desired_turn_rate, MC_STEP_MODE_AUTO);
    set_velocity_with_stall_detection(MC_RIGHT_MOTOR, power - desired_turn_rate, MC_STEP_MODE_AUTO);

    // store filtered and unfiltered velocity error values for next iteration
    prev_vel = vel_error;
    prev_vel_error_lpf = vel_error_lpf;
}

// reset balance control
void balance_control_reset()
{
    bp_i = 0;
    vel_error_lpf = 0;

    prev_vel = 0;
    prev_vel_error_lpf = 0;
    balance_point_power = 0;
}

// initialize balance control
void balance_control_init()
{
    // output power to motors
    balance_point_power = 0;

    // low pass filtered velocity error of the previous cycle
    prev_vel_error_lpf = 0;

    // I component of the top layer of the balancing algorithm
    bp_i = 0;

    // low pass filtered velocity error of the current cycle
    vel_error_lpf;
}
