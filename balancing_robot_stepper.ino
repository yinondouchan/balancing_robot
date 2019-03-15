
#include <Arduino.h>

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1

#define STEP_MODE_FULL 1
#define STEP_MODE_HALF 2
#define STEP_MODE_QUARTER 4
#define STEP_MODE_EIGHTH 8
#define STEP_MODE_ONE_SIXTEENTH 16

#define DIGITAL_POT_WRITE_CMD 0x00
#define LOOP_COUNTER_DIVISOR 200

#define DT_MICROS 4000
#define FULL_STEPS_PER_REV 200

#define LEFT_MOTOR_STEP_PIN 4
#define LEFT_MOTOR_DIR_PIN 2
#define LEFT_MOTOR_MS1_PIN 6
#define LEFT_MOTOR_MS2_PIN 5
#define LEFT_MOTOR_MS3_PIN 3

#define RIGHT_MOTOR_STEP_PIN 8
#define RIGHT_MOTOR_DIR_PIN 7
#define RIGHT_MOTOR_MS1_PIN 11
#define RIGHT_MOTOR_MS2_PIN 12
#define RIGHT_MOTOR_MS3_PIN 10

#define ENCODER_FULL_SCALE 1024

#define STALL_SPEED_DIFF_THRESHOLD 50

#define CLOCK_SPEED_HZ 16000000

unsigned long timestamp;

// timer counter for left and right motors
volatile int32_t left_motor_counter;
volatile int32_t right_motor_counter;

// timer for main loop
volatile int32_t loop_counter;

// timer ticks per motor step
int32_t left_motor_nticks;
int32_t right_motor_nticks;

// motor's step velocity in full-steps per second
int32_t left_motor_desired_vel;
int32_t left_motor_actual_step_vel;
int32_t right_motor_desired_vel;
int32_t right_motor_actual_step_vel;

// motor's encoder velocity in full-steps per second
float left_motor_encoder_vel;
float right_motor_encoder_vel;

// previous angle read by encoder
int16_t left_motor_prev_angle;
int16_t right_motor_prev_angle;

int32_t ctr;

void setup_timers()
{
    cli();

    // initialize Timer0 for controlling right stepper motor
    TCCR0A = 0;    // set entire TCCR0A register to 0
    TCCR0B = 0;    // set entire TCCR0B register to 0 
                   // (as we do not know the initial  values)

    TIMSK0 |= (1 << OCIE0A);

    TCCR0B |= _BV(CS01);
    OCR0A = 255;
    TCCR0A |= _BV(WGM01);

    // initialize Timer1 for main loop
    TCCR1A = 0;    // set entire TCCR1A register to 0
    TCCR1B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    TIMSK1 |= (1 << OCIE1A);

    // set CTC mode and set prescaler to 8
    TCCR1B |= _BV(CS11) | _BV(WGM12) | _BV(WGM13);
    ICR1 = 39;

    // initialize Timer2 for controlling left stepper motor
    TCCR2A = 0;    // set entire TCCR2A register to 0
    TCCR2B = 0;    // set entire TCCR2B register to 0 
                   // (as we do not know the initial  values)

    TIMSK2 |= (1 << OCIE2A);
    TCCR2B |= _BV(CS21);
    OCR2A = 255;
    TCCR2A |= _BV(WGM21);
    
    // enable global interrupts:
    sei();
}

ISR(TIMER1_COMPA_vect)
{
    loop_counter = (loop_counter >= LOOP_COUNTER_DIVISOR) ? 0 : loop_counter + 1;
}

ISR(TIMER2_COMPA_vect)
{
    // called every compare-match
    digitalWrite(LEFT_MOTOR_STEP_PIN, left_motor_counter >= (left_motor_nticks - 1));
    left_motor_counter = (left_motor_counter >= (left_motor_nticks - 1)) ? 0 : left_motor_counter + 1;
}

ISR(TIMER0_COMPA_vect)
{
    digitalWrite(RIGHT_MOTOR_STEP_PIN, right_motor_counter >= (right_motor_nticks - 1));
    right_motor_counter = (right_motor_counter >= (right_motor_nticks - 1)) ? 0 : right_motor_counter + 1;
}

void setup()
{
    setup_timers();
    
    Serial.begin(115200);

    // set up STEP and DIR pins
    pinMode(LEFT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_MS2_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_MS2_PIN, OUTPUT);
    
    digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);
    digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_STEP_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);

    // encoder
    pinMode(A0, INPUT);

    loop_counter = 0;

    // set up left motor
    left_motor_nticks = 0;
    left_motor_counter = 0;
    left_motor_desired_vel = 0;
    left_motor_actual_step_vel = 0;
    left_motor_encoder_vel = 0;
    left_motor_prev_angle = analogRead(A0);

    // set up right motir
    right_motor_nticks = 0;
    right_motor_counter = 0;
    right_motor_desired_vel = 0;
    right_motor_actual_step_vel = 0;
    right_motor_encoder_vel = 0;

    ctr = 0;
}

void read_encoder_velocity()
{
    int16_t new_angle = analogRead(A0);

    // calculate encoder angle difference
    int16_t left_encoder_angle_diff = (new_angle - left_motor_prev_angle);

    // deal with angle overflow, i.e. transition from ENCODER_FULL_SCALE - 1 to 0 or backwards
    if (abs(left_encoder_angle_diff) > ENCODER_FULL_SCALE/2)
    {
        // angle_diff should be positive
        if (new_angle < left_motor_prev_angle) left_encoder_angle_diff += ENCODER_FULL_SCALE;
        // angle_diff should be negative
        else left_encoder_angle_diff -= ENCODER_FULL_SCALE;
    }

    // find velocity in encoder step per second
    float new_motor_encoder_vel = 1000000.0 * (float)left_encoder_angle_diff / DT_MICROS;
    // convert to full-steps per second
    new_motor_encoder_vel = new_motor_encoder_vel * FULL_STEPS_PER_REV / ENCODER_FULL_SCALE;

    // add a low-pass filter
    left_motor_encoder_vel = 0.9 * left_motor_encoder_vel + 0.1 * new_motor_encoder_vel;
    
    left_motor_prev_angle = new_angle;
}

// set the velocity of a motor with a specific step_mode (full step or microstepping)
void set_velocity(uint8_t motor, float velocity, uint8_t step_mode)
{
    if (motor == LEFT_MOTOR)
    {
        if (step_mode == STEP_MODE_FULL)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_HALF)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_QUARTER)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, LOW);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_EIGHTH)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_ONE_SIXTEENTH)
        {
            digitalWrite(LEFT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(LEFT_MOTOR_MS3_PIN, HIGH);
        }
        
        set_step_rate_left(velocity * step_mode);
    }
    if (motor == RIGHT_MOTOR)
    {
        if (step_mode == STEP_MODE_FULL)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_HALF)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_QUARTER)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, LOW);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_EIGHTH)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, LOW);
        }
        else if (step_mode == STEP_MODE_ONE_SIXTEENTH)
        {
            digitalWrite(RIGHT_MOTOR_MS1_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS2_PIN, HIGH);
            digitalWrite(RIGHT_MOTOR_MS3_PIN, HIGH);
        }
        
        set_step_rate_right(velocity * step_mode);
    }
}

void set_step_rate_left(float velocity)
{
    if (velocity == 0)
    {
        OCR0A = 255;
        left_motor_nticks = 1;
        return;
    }
    // determine highest nticks such that OCR can be lower than 256
    int32_t max_n_ticks = max(ceil((float)CLOCK_SPEED_HZ / (256 * 8 * velocity)), 2);
    left_motor_nticks = max_n_ticks;
    OCR2A = CLOCK_SPEED_HZ / (max_n_ticks * 8 * velocity) - 1;
}

void set_step_rate_right(float velocity)
{

    if (velocity == 0)
    {
        OCR0A = 255;
        right_motor_nticks = 1;
        return;
    }
    
    // determine highest nticks such that OCR can be lower than 256
    int32_t max_n_ticks = max(ceil((float)CLOCK_SPEED_HZ / (256 * 8 * velocity)), 2);
    right_motor_nticks = max_n_ticks;
    OCR0A = CLOCK_SPEED_HZ / (max_n_ticks * 8 * velocity) - 1;
}

void loop()
{
    // busy-wait until next iteration for a 250Hz loop rate
    set_velocity(LEFT_MOTOR, min(ctr, 200), STEP_MODE_ONE_SIXTEENTH);
    set_velocity(RIGHT_MOTOR, min(ctr, 200), STEP_MODE_ONE_SIXTEENTH);
    //Serial.println(ctr);
    //if ((ctr % 2000) > 1000) set_velocity(LEFT_MOTOR, 200, STEP_MODE_FULL);
    //else set_velocity(LEFT_MOTOR, 200, STEP_MODE_QUARTER);
    ctr += 5;
    while(loop_counter < LOOP_COUNTER_DIVISOR);
}
