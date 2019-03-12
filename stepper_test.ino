#include <Arduino.h>
#include <SPI.h>

#define DIGITAL_POT_WRITE_CMD 0x00
#define LOOP_COUNTER_DIVISOR 200

#define DT_MICROS 4000
#define FULL_STEPS_PER_REV 200

#define LEFT_MOTOR_STEP_PIN 4
#define LEFT_MOTOR_DIR_PIN 2

#define ENCODER_FULL_SCALE 1024

#define STALL_SPEED_DIFF_THRESHOLD 50

#define STEP_FREQ_HZ 50000

unsigned long timestamp;

volatile int32_t left_motor_counter;
volatile int32_t loop_counter;
bool main_loop_triggered;
int32_t left_motor_nticks;

// motor's step velocity in full-steps per second
int32_t left_motor_desired_vel;
int32_t left_motor_actual_step_vel;

// motor's encoder velocity in full-steps per second
float left_motor_encoder_vel;

int16_t left_motor_prev_angle;

void setup_timers()
{
    cli();
  
    // initialize Timer1
    TCCR1A = 0;    // set entire TCCR1A register to 0
    TCCR1B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    TIMSK1 |= (1 << OCIE1A);

    // no prescaler by default
    TCCR1B |= _BV(CS10) | _BV(WGM12) | _BV(WGM13);
    ICR1 = 65535;

    // initialize Timer2
    TCCR2A = 0;    // set entire TCCR2A register to 0
    TCCR2B = 0;    // set entire TCCR2B register to 0 
                   // (as we do not know the initial  values)

    TIMSK2 |= (1 << OCIE2A);

    // no prescaler by default
    TCCR2B |= _BV(CS21);
    OCR2A = 39;
    TCCR2A |= _BV(WGM21);
    
    // enable global interrupts:
    sei();
}

ISR(TIMER1_COMPA_vect)
{
    // called every compare-match
    digitalWrite(LEFT_MOTOR_STEP_PIN, left_motor_counter >= (left_motor_nticks - 1));
    left_motor_counter = (left_motor_counter >= (left_motor_nticks - 1)) ? 0 : left_motor_counter + 1;

}

ISR(TIMER2_COMPA_vect)
{
    // called every 20 us
    loop_counter = (loop_counter >= LOOP_COUNTER_DIVISOR) ? 0 : loop_counter + 1;
}

void setup()
{
    setup_timers();
    
    Serial.begin(115200);

    // STEP pin
    pinMode(LEFT_MOTOR_STEP_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    digitalWrite(LEFT_MOTOR_STEP_PIN, LOW);

    // encoder
    pinMode(A0, INPUT);

    left_motor_nticks = 0;
    left_motor_counter = 0;
    left_motor_desired_vel = 0;
    left_motor_actual_step_vel = 0;
    loop_counter = 0;

    left_motor_encoder_vel = 0;
    left_motor_prev_angle = analogRead(A0);

    main_loop_triggered = false;
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

void set_velocity(int32_t velocity)
{
    left_motor_desired_vel = velocity;
    left_motor_nticks = STEP_FREQ_HZ / velocity;

    // since the above nticks is a division of two integers we won't necessarily obtain the desired velocity but rather a value close to it.
    left_motor_actual_step_vel = STEP_FREQ_HZ / left_motor_nticks;
}

void set_velocity2(int32_t velocity)
{
    if (velocity < 250)
    {
        // in slow velocities 
        // clock divisor - 64
        TCCR1B |= _BV(CS11);
        //ICR1 = 16000000 / max(64 * 2 * velocity, 256);
        ICR1 = 20;
        left_motor_nticks = 16000000 / (64 * velocity * ICR1);
    }
    else
    {
        left_motor_nticks = 2;
        // clock divisor - 1
        TCCR1B &= ~_BV(CS11);
        ICR1 = max(16000000 / (2 * velocity), 511);
    }
}

void handle_stall()
{
    set_velocity(left_motor_encoder_vel);
}

void loop()
{
    if (!main_loop_triggered && (loop_counter == 0))
    {
        main_loop_triggered = true;

        //digitalWrite(DIR_PIN, HIGH);
        set_velocity2(min(millis() / 5, 13000));       
        read_encoder_velocity();
        Serial.println(left_motor_encoder_vel);

        /*if (abs(motor_actual_step_vel - motor_encoder_vel)) 
        {
            handle_stall();
        }
        else
        {
            set_velocity(1000);
        }
        
        Serial.println(motor_encoder_vel);*/
    }
    else if (loop_counter != 0)
    {
        main_loop_triggered = false;
    }
}
