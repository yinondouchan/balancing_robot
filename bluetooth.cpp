#include "bluetooth.h"

// joystick commands come in packs of 6 bytes
char buf[JOYSTICK_BUF_SIZE];

// state of reading joystick data (byte number)
uint8_t buf_state;

// robot's control velocity and angular velocity
int bt_desired_vel, bt_desired_vel_diff;

// balance angle trimming (milli-deg)
int bt_trim_angle;

int8_t bt_joystick_vel_sensitivity;

// callback for trim event
trim_callback_t trim_callback;

// callback for emergency stop event
estop_callback_t estop_callback;

enum command_type cmd_type;

void bt_init()
{
    // assume Serial was initialized on 9600 baud

    bt_desired_vel = 0;
    bt_desired_vel_diff = 0;
    bt_trim_angle = 0;
    bt_joystick_vel_sensitivity = JOYSTICK_VEL_SENSITIVITY;
}

void bt_read_joystick_control()
{
    // try reading a byte from bluetooth.
    int in_byte = 0;
    uint8_t buf_size;
    while (Serial.available() > 0)
    {
        in_byte = Serial.read();
        // if first byte of packet is read determine whether it is a joystick command or a button command
        if (buf_state == 0)
        {
            if (in_byte == 'S')
            {
                cmd_type = CMD_JOYSTICK;
                buf_size = JOYSTICK_BUF_SIZE;
            }
            if (in_byte == 'B')
            {
                cmd_type = CMD_BUTTON;
                buf_size = BUTTON_BUF_SIZE;
            }
        }
        
        buf[buf_state] = in_byte;
        buf_state += 1;

        if (buf_state == buf_size)
        {
            (cmd_type == CMD_BUTTON) ? bt_decode_button_packet() : bt_decode_joystick_control_packet();
            buf_state = 0;
        }
    }
}

void bt_decode_joystick_control_packet()
{
    // read the x and y axes of the joystick. Each should return values between 0 (left/down) and 1024 (right/up) where 512 is centered
    uint16_t x_reading = 1000 * (buf[1] - '0') + 100 * (buf[2] - '0') + 10 * (buf[3] - '0') + (buf[4] - '0');
    uint16_t y_reading = 1000 * (buf[8] - '0') + 100 * (buf[9] - '0') + 10 * (buf[10] - '0') + (buf[11] - '0');

    bt_desired_vel = (x_reading - 512) * bt_joystick_vel_sensitivity;
    bt_desired_vel_diff = (y_reading - 512) * JOYSTICK_ANG_VEL_SENSITIVITY;
}

void add_to_trim_angle(int increment)
{
    bt_trim_angle += increment;
    (*trim_callback)(bt_trim_angle);
}

void bt_decode_button_packet()
{
    char button_num = buf[1];

    switch(button_num)
    {
        case '1':
        bt_joystick_vel_sensitivity += 1;
        break;
        case '2':
        bt_joystick_vel_sensitivity -= 1;
        bt_joystick_vel_sensitivity = bt_joystick_vel_sensitivity < 0 ? 0 : bt_joystick_vel_sensitivity;
        break;
        case '3':
        (*estop_callback)();
        break;
    }
}

void bt_set_trim_callback(trim_callback_t func)
{
    trim_callback = func;
}

void bt_set_estop_callback(estop_callback_t func)
{
    estop_callback = func;
}
