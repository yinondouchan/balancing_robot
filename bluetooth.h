#ifndef BLUETOOTH_H
#define BLUETOOTH_H

/*
 * Bluetooth stuff
 */

#include <Arduino.h>

#define JOYSTICK_BUF_SIZE 14
#define BUTTON_BUF_SIZE 4
#define MAX_BUF_SIZE 14

// velocity sensitivity per unit
#define JOYSTICK_VEL_SENSITIVITY 1

// angular velocity sensitivity per unit
#define JOYSTICK_ANG_VEL_SENSITIVITY 2

#define TRIM_INCREMENT_MILLIDEG 300

// trimming of balance angle (i.e. manual fine tuning)
typedef void(*trim_callback_t)(int);

// emergency stop callback
typedef void(*estop_callback_t)();

// parameter update callback
typedef void(*param_callback_t)(uint8_t, uint8_t, uint8_t*);

// robot's control velocity and angular velocity
extern int32_t bt_desired_vel, bt_desired_vel_diff;
extern float bt_desired_vel_diff_lpf;

// balance angle trimming (milli-deg)
extern int bt_trim_angle;

enum command_type {CMD_JOYSTICK, CMD_BUTTON, CMD_PARAM};
enum read_state {READ_CMD, HANDLE_JOYSTICK_CMD, HANDLE_BUTTON_CMD, HANDLE_PARAM_CMD};

// initialize anything related to bluetooth
void bt_init();

void bt_read();

// read serial input from the Joystick BT Commander app from Android
void bt_read_joystick_control();

// given a packet from serial decode it and translate it to desired velocity
void bt_decode_joystick_control_packet();

void bt_decode_button_packet();

void bt_set_trim_callback(trim_callback_t func);

void bt_set_estop_callback(estop_callback_t func);

void bt_set_param_callback(param_callback_t func);

void bt_decode_param(const char *param_name, const uint8_t len);

#endif // BLUETOOTH_H
