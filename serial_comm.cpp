#include "serial_comm.h"

// joystick commands come in packs of 14 bytes
uint8_t buf[JOYSTICK_BUF_SIZE];

// state of reading joystick data (byte number relative to command's first byte)
uint8_t buf_state;

// robot's control velocity and angular velocity
int32_t serial_comm_desired_vel, serial_comm_desired_vel_diff;
float serial_comm_desired_vel_diff_lpf;

int8_t serial_comm_joystick_vel_sensitivity;

// callback for emergency stop event
estop_callback_t estop_callback;

// callback for remote parameter tuning
param_callback_t param_callback;

enum command_type cmd_type;
enum read_state read_state;

// parameter value length in a tlv
uint8_t param_length;

void serial_comm_init()
{
    // assume Serial was initialized on 9600 baud

    serial_comm_desired_vel = 0;
    serial_comm_desired_vel_diff = 0;
    serial_comm_joystick_vel_sensitivity = JOYSTICK_VEL_SENSITIVITY;
    read_state = READ_CMD;
}

void serial_comm_read()
{
    int in_byte = 0;
    if (read_state == READ_CMD)
    {
        // read only one byte since all the following bytes are to be handled by the command handler
        if (Serial.available() > 0)
        {
            in_byte = Serial.read();
            switch(in_byte)
            {
                case 'P':
                cmd_type = CMD_PARAM;
                read_state = HANDLE_PARAM_CMD;
                break;
                case 'S':
                cmd_type = CMD_JOYSTICK;
                read_state = HANDLE_JOYSTICK_CMD;
                break;
                case 'B':
                cmd_type = CMD_BUTTON;
                read_state = HANDLE_BUTTON_CMD;
                break;
            }
            
            buf[buf_state] = in_byte;
            buf_state += 1;
        }
    }
    if (read_state == HANDLE_JOYSTICK_CMD)
    {
        while (Serial.available() > 0)
        {
            in_byte = Serial.read();
            buf[buf_state] = in_byte;
            buf_state += 1;

            if (buf_state == JOYSTICK_BUF_SIZE)
            {
                serial_comm_decode_joystick_control_packet();
                buf_state = 0;
                read_state = READ_CMD;
                break;
            }
        }
    }
    if (read_state == HANDLE_BUTTON_CMD)
    {
        while (Serial.available() > 0)
        {
            in_byte = Serial.read();
            buf[buf_state] = in_byte;
            buf_state += 1;

            if (buf_state == BUTTON_BUF_SIZE)
            {
                serial_comm_decode_button_packet();
                buf_state = 0;
                read_state = READ_CMD;
                break;
            }
        }
    }
    if (read_state == HANDLE_PARAM_CMD)
    {
        while (Serial.available() > 0)
        {
            in_byte = Serial.read();
            buf[buf_state] = in_byte;
            buf_state += 1;

            // When buf index is 1 the tag value is read. When buf index is 2, length value is read and afterwards the value itself is read.
            if (buf_state == 3) param_length = in_byte;
            if (buf_state == 3 + param_length)
            {
                // done reading the TLV's value - call the parameter callback with the obtained TLV
                (*param_callback)(buf[1], buf[2], buf + 3);
                buf_state = 0;
                read_state = READ_CMD;
                break;
            }
        }
    }
}

void serial_comm_read_joystick_control()
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
            (cmd_type == CMD_BUTTON) ? serial_comm_decode_button_packet() : serial_comm_decode_joystick_control_packet();
            buf_state = 0;
        }
    }
}

void serial_comm_decode_joystick_control_packet()
{
    // read the x and y axes of the joystick. Each should return values between 0 (left/down) and 1024 (right/up) where 512 is centered
    int16_t x_reading = 1000 * (buf[1] - '0') + 100 * (buf[2] - '0') + 10 * (buf[3] - '0') + (buf[4] - '0');
    int16_t y_reading = 1000 * (buf[8] - '0') + 100 * (buf[9] - '0') + 10 * (buf[10] - '0') + (buf[11] - '0');

    serial_comm_desired_vel = (x_reading - 512) * serial_comm_joystick_vel_sensitivity;

    // add some low-pass filtering to angular velocity
    float turn_rate_attenuation = (1.0 + float(abs(serial_comm_desired_vel)) / TURN_RATE_ATTENUATION_FACTOR); // make the robot's turn less sharp the faster it goes
    serial_comm_desired_vel_diff = (y_reading - 512) * JOYSTICK_ANG_VEL_SENSITIVITY / turn_rate_attenuation;
}

void serial_comm_decode_button_packet()
{
    char button_num = buf[1];

    switch(button_num)
    {
        case '1':
        serial_comm_joystick_vel_sensitivity += 1;
        break;
        case '2':
        serial_comm_joystick_vel_sensitivity -= 1;
        serial_comm_joystick_vel_sensitivity = serial_comm_joystick_vel_sensitivity < 0 ? 0 : serial_comm_joystick_vel_sensitivity;
        break;
        case '3':
        (*estop_callback)();
        break;
    }
}

void serial_comm_set_estop_callback(estop_callback_t func)
{
    estop_callback = func;
}

void serial_comm_set_param_callback(param_callback_t func)
{
    param_callback = func;
}