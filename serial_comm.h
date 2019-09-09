#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

/*
 * Serial interface stuff
 */

#include <Arduino.h>

// throttle and rotation control packet size
#define JOYSTICK_BUF_SIZE 14

// button control packet size
#define BUTTON_BUF_SIZE 4

// velocity sensitivity per unit
#define JOYSTICK_VEL_SENSITIVITY 2

// angular velocity sensitivity per velocity command unit
#define JOYSTICK_ANG_VEL_SENSITIVITY 2.2

#define TURN_RATE_ATTENUATION_FACTOR 1500

// emergency stop callback
typedef void(*estop_callback_t)();

// parameter update callback
typedef void(*param_callback_t)(uint8_t, uint8_t, uint8_t*);

// robot's control velocity and angular velocity
extern int32_t serial_comm_desired_vel, serial_comm_desired_vel_diff;
extern float serial_comm_desired_vel_diff_lpf;

// remote control command types
enum command_type {CMD_JOYSTICK, CMD_BUTTON, CMD_PARAM};

// serial packet read states:
// READ_CMD: initial state waiting to receive an actual command
// HANDLE_JOYSTICK_CMD: handle a joystick command (throttle and turn)
// HANDLE_BUTTON_CMD: handles a button command
// HANDLE_PARAM_CMD: handles a parameter update command
enum read_state {READ_CMD, HANDLE_JOYSTICK_CMD, HANDLE_BUTTON_CMD, HANDLE_PARAM_CMD};

// initialize anything related to bluetooth
void serial_comm_init();

// read a serial packet
void serial_comm_read();

// read serial input from the Joystick BT Commander app from Android
void serial_comm_read_joystick_control();

// given a packet from serial decode it and translate it to desired velocity
void serial_comm_decode_joystick_control_packet();

// decode a button packet
void serial_comm_decode_button_packet();

// set callback for handling an emergency stop
void serial_comm_set_estop_callback(estop_callback_t func);

// set callback for updating a parameter
void serial_comm_set_param_callback(param_callback_t func);

// decode a parameter
void serial_comm_decode_param(const char *param_name, const uint8_t len);

#endif // SERIAL_COMM_H
