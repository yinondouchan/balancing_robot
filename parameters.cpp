#include <Arduino.h>

#include "parameters.h"

parameters_t parameters;

// set default tunable parameters
void parameters_set_default()
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
void parameters_set_parameter(uint8_t tag, uint8_t len, uint8_t *value)
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
