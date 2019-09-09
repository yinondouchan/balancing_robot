#include "serial_comm.h"

// joystick commands come in packs of 14 bytes
uint8_t buf[JOYSTICK_BUF_SIZE];

// state of reading joystick data (byte number relative to command's first byte)
uint8_t buf_state;

// robot's control velocity and angular velocity
int32_t serial_comm_desired_vel, serial_comm_desired_vel_diff;
float serial_comm_desired_vel_diff_lpf;

// joystick velocity sensitivity (full-steps per seconds per one unit change in input)
int8_t serial_comm_joystick_vel_sensitivity;

// callback for emergency stop event
estop_callback_t estop_callback;

// callback for remote parameter tuning
param_callback_t param_callback;

// command type
enum command_type cmd_type;

// serial packet read states:
// READ_CMD: initial state waiting to receive an actual command
// HANDLE_JOYSTICK_CMD: handle a joystick command (throttle and turn)
// HANDLE_BUTTON_CMD: handles a button command
// HANDLE_PARAM_CMD: handles a parameter update command
enum read_state read_state;

// parameter value length in a tlv
uint8_t param_length;

// initialize anything related to bluetooth
void serial_comm_init()
{
    // assume Serial was initialized on 9600 baud

    // initialized output linear and angular velocities (full-steps/sec)
    serial_comm_desired_vel = 0;
    serial_comm_desired_vel_diff = 0;

    // set initial joystick linear velocity sensitivity
    serial_comm_joystick_vel_sensitivity = JOYSTICK_VEL_SENSITIVITY;

    // initial read state
    read_state = READ_CMD;
}

// read a serial packet
void serial_comm_read()
{
    int in_byte = 0;

    if (read_state == READ_CMD)   
    {
        // handle first byte of a commandd
        if (Serial.available() > 0)
        {
            // read the command byte
            in_byte = Serial.read();

            // decode the command byte
            switch(in_byte)
            {
                case 'P':
                // parameter update command
                cmd_type = CMD_PARAM;
                read_state = HANDLE_PARAM_CMD;
                break;
                case 'S':
                // joystick control command
                cmd_type = CMD_JOYSTICK;
                read_state = HANDLE_JOYSTICK_CMD;
                break;
                case 'B':
                // button command
                cmd_type = CMD_BUTTON;
                read_state = HANDLE_BUTTON_CMD;
                break;
            }

            // fill buffer with this byte
            buf[buf_state] = in_byte;

            // increment byte index
            buf_state += 1;
        }
    }
    if (read_state == HANDLE_JOYSTICK_CMD)
    {
        // handle joystick command
        while (Serial.available() > 0)
        {
            // read the byte
            in_byte = Serial.read();

            // put the byte in the command buffer
            buf[buf_state] = in_byte;

            // increment command buffer index
            buf_state += 1;

            // check if command read is complete
            if (buf_state == JOYSTICK_BUF_SIZE)
            {
                // decode command
                serial_comm_decode_joystick_control_packet();
                buf_state = 0;
                read_state = READ_CMD;
                break;
            }
        }
    }
    if (read_state == HANDLE_BUTTON_CMD)
    {
        // handle button command
        while (Serial.available() > 0)
        {
            // read the byte
            in_byte = Serial.read();

            // put the byte in the command buffer
            buf[buf_state] = in_byte;

            // increment command buffer index
            buf_state += 1;

            // check if command read is complete
            if (buf_state == BUTTON_BUF_SIZE)
            {
                // decode command
                serial_comm_decode_button_packet();
                buf_state = 0;
                read_state = READ_CMD;
                break;
            }
        }
    }
    if (read_state == HANDLE_PARAM_CMD)
    {
        // handle parameter update command
        while (Serial.available() > 0)
        {
            // read the byte
            in_byte = Serial.read();

            // put the byte in the command buffer
            buf[buf_state] = in_byte;

            // parameter command comes in the form of tag-length-value (TLV) elements
            // when buf index is 1 the tag value is read. When buf index is 2, length value is read and afterwards the value itself is read.

            // obtain parmeter length in bytes from length field
            if (buf_state == PARAM_CMD_TLV_LENGTH_INDEX) param_length = in_byte;

            if (buf_state == PARAM_CMD_TLV_VALUE_INDEX + param_length - 1)
            {
                // done reading the TLV's value - call the parameter callback with the value contained in the TLV
                (*param_callback)(buf[PARAM_CMD_TLV_TAG_INDEX], buf[PARAM_CMD_TLV_LENGTH_INDEX], buf + PARAM_CMD_TLV_VALUE_INDEX);
                buf_state = 0;
                read_state = READ_CMD;
                break;
            }

            // increment command buffer index
            buf_state += 1;
        }
    }
}

// read serial input from the Joystick BT Commander app from Android
void serial_comm_read_joystick_control()
{
    // try reading a byte from bluetooth.
    int in_byte = 0;
    uint8_t buf_size;
    while (Serial.available() > 0)
    {
        // read the byte
        in_byte = Serial.read();
        
        // if first byte of packet is read determine whether it is a joystick command or a button command
        if (buf_state == 0)
        {
            if (in_byte == 'S')
            {
                // command is throttle/turn_rate
                cmd_type = CMD_JOYSTICK;
                buf_size = JOYSTICK_BUF_SIZE;
            }
            if (in_byte == 'B')
            {
                // command is button
                cmd_type = CMD_BUTTON;
                buf_size = BUTTON_BUF_SIZE;
            }
        }

        // put byte in byte buffer and increment buffer index
        buf[buf_state] = in_byte;
        buf_state += 1;

        if (buf_state == buf_size)
        {
            // decode command accordingly when finished reading it
            (cmd_type == CMD_BUTTON) ? serial_comm_decode_button_packet() : serial_comm_decode_joystick_control_packet();
            buf_state = 0;
        }
    }
}

// given a packet from serial decode it and translate it to desired velocity
void serial_comm_decode_joystick_control_packet()
{
    // read the x and y axes of the joystick. Each should return values between 0 (left/down) and 1024 (right/up) where 512 is centered
    int16_t x_reading = 1000 * (buf[1] - '0') + 100 * (buf[2] - '0') + 10 * (buf[3] - '0') + (buf[4] - '0');
    int16_t y_reading = 1000 * (buf[8] - '0') + 100 * (buf[9] - '0') + 10 * (buf[10] - '0') + (buf[11] - '0');

    // set desired velocity in full-steps per second
    serial_comm_desired_vel = (x_reading - 512) * serial_comm_joystick_vel_sensitivity;

    // turn rate has discrete level and this causes the steering to sound annoying. Therefore, add some low-pass filtering to angular velocity
    float turn_rate_attenuation = (1.0 + float(abs(serial_comm_desired_vel)) / TURN_RATE_ATTENUATION_FACTOR); // make the robot's turn less sharp the faster it goes

    // set desired turn rate
    serial_comm_desired_vel_diff = (y_reading - 512) * JOYSTICK_ANG_VEL_SENSITIVITY / turn_rate_attenuation;
}

// decode a button packet
void serial_comm_decode_button_packet()
{
    char button_num = buf[1];

    switch(button_num)
    {
        case '1':
        // increase velocity sensitivity (full-steps/sec per input throttle unit)
        serial_comm_joystick_vel_sensitivity += 1;
        break;
        case '2':
        // decrease velocity sensitivity (full-steps/sec per input throttle unit)
        serial_comm_joystick_vel_sensitivity -= 1;

        // do not decrease sensitivity to negative values
        serial_comm_joystick_vel_sensitivity = serial_comm_joystick_vel_sensitivity < 0 ? 0 : serial_comm_joystick_vel_sensitivity;
        break;
        case '3':
        // emergency stop
        (*estop_callback)();
        break;
    }
}

// set callback for handling an emergency stop
void serial_comm_set_estop_callback(estop_callback_t func)
{
    estop_callback = func;
}

// set callback for updating a parameter
void serial_comm_set_param_callback(param_callback_t func)
{
    param_callback = func;
}
