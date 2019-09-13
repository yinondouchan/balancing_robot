// motor index
#define ENCODERS_LEFT_ENCODER 0
#define ENCODERS_RIGHT_ENCODER 1

// left motor encoder pin
#define ENCODERS_LEFT_MOTOR_PIN A0

// right motor encoder pin
#define ENCODERS_RIGHT_MOTOR_PIN A1

// number of encoder values per full revolution of the motor
#define ENCODER_FULL_SCALE 1024

// motor's encoder velocity in full-steps per second
extern float encoders_left_motor_vel;
extern float encoders_right_motor_vel;

// read velocity from an encoder
void encoders_read_velocity(uint8_t encoder);

// initialize encoders
void encoders_init();
