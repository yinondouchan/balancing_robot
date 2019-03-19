#ifndef BLUETOOTH_H
#define BLUETOOTH_H

/*
 * Bluetooth stuff
 */

#include <Arduino.h>

#define JOYSTICK_BUF_SIZE 14
#define BUTTON_BUF_SIZE 4

// velocity sensitivity per unit
#define JOYSTICK_VEL_SENSITIVITY 1

// angular velocity sensitivity per unit
#define JOYSTICK_ANG_VEL_SENSITIVITY 3

#define TRIM_INCREMENT_MILLIDEG 300

// trimming of balance angle (i.e. manual fine tuning)
typedef void(*trim_callback_t)(int);

// emergency stop callback
typedef void(*estop_callback_t)();

// robot's control velocity and angular velocity
extern int bt_desired_vel, bt_desired_vel_diff;

// balance angle trimming (milli-deg)
extern int bt_trim_angle;

enum command_type {CMD_JOYSTICK, CMD_BUTTON};

// initialize anything related to bluetooth
void bt_init();

// read serial input from the Joystick BT Commander app from Android
void bt_read_joystick_control();

// given a packet from serial decode it and translate it to desired velocity
void bt_decode_joystick_control_packet();

void bt_decode_button_packet();

void bt_set_trim_callback(trim_callback_t func);

void bt_set_estop_callback(estop_callback_t func);

void decode_param(const char *param_name, const uint8_t len);

#endif // BLUETOOTH_H
