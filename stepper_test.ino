#include <Arduino.h>
#include <SPI.h>

#define DIGITAL_POT_WRITE_CMD 0x00
#define LOOP_COUNTER_DIVISOR 200

#define DT_MICROS 4000
#define FULL_STEPS_PER_REV 200

#define ENCODER_FULL_SCALE 1024

#define STALL_SPEED_DIFF_THRESHOLD 50

#define STEP_FREQ_HZ 50000

unsigned long timestamp;

volatile int32_t motor_counter;
volatile int32_t loop_counter;
bool main_loop_triggered;
int32_t motor_nticks;

// motor's step velocity in full-steps per second
int32_t motor_desired_vel;
int32_t motor_actual_step_vel;

// motor's encoder velocity in full-steps per second
float motor_encoder_vel;

int16_t motor_prev_angle;

void setup_timers()
{
    cli();
  
    // initialize Timer2
    TCCR2A = 0;    // set entire TCCR1A register to 0
    TCCR2B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    TIMSK2 |= (1 << OCIE2A);
    TCCR2B |= (1 << CS21);
    OCR2A = 39;
    TCCR2A |= (1 << WGM21);
    
    // enable global interrupts:
    sei();
}

ISR(TIMER2_COMPA_vect)
{
    // called about once in every 20 us
    PORTB = 0b00000000 | (motor_counter >= (motor_nticks - 1));
    motor_counter = (motor_counter >= (motor_nticks - 1)) ? 0 : motor_counter + 1;
    loop_counter = (loop_counter >= LOOP_COUNTER_DIVISOR) ? 0 : loop_counter + 1;
}

void setup()
{
    setup_timers();
    
    Serial.begin(115200);

    // STEP pin
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    // encoder
    pinMode(A0, INPUT);

    motor_nticks = 0;
    motor_counter = 0;
    motor_desired_vel = 0;
    motor_actual_step_vel = 0;
    loop_counter = 0;

    motor_encoder_vel = 0;
    motor_prev_angle = analogRead(A0);

    main_loop_triggered = false;
}

void read_encoder_velocity()
{
    int16_t new_angle = analogRead(A0);

    // calculate encoder angle difference
    int16_t encoder_angle_diff = (new_angle - motor_prev_angle);

    // deal with angle overflow, i.e. transition from ENCODER_FULL_SCALE - 1 to 0 or backwards
    if (abs(encoder_angle_diff) > ENCODER_FULL_SCALE/2)
    {
        // angle_diff should be positive
        if (new_angle < motor_prev_angle) encoder_angle_diff += ENCODER_FULL_SCALE;
        // angle_diff should be negative
        else encoder_angle_diff -= ENCODER_FULL_SCALE;
    }

    // find velocity in encoder step per second
    float new_motor_encoder_vel = 1000000.0 * (float)encoder_angle_diff / DT_MICROS;
    // convert to full-steps per second
    new_motor_encoder_vel = new_motor_encoder_vel * FULL_STEPS_PER_REV / ENCODER_FULL_SCALE;

    // add a low-pass filter
    motor_encoder_vel = 0.9 * motor_encoder_vel + 0.1 * new_motor_encoder_vel;
    
    motor_prev_angle = new_angle;
}

void set_velocity(int32_t velocity)
{
    motor_desired_vel = velocity;
    motor_nticks = STEP_FREQ_HZ / velocity;

    // since the above nticks is a division of two integers we won't necessarily obtain the desired velocity but rather a value close to it.
    motor_actual_step_vel = STEP_FREQ_HZ / motor_nticks;
}

void handle_stall()
{
    set_velocity(motor_encoder_vel);
}

void loop()
{
    if (!main_loop_triggered && (loop_counter == 0))
    {
        main_loop_triggered = true;
        
        read_encoder_velocity();

        if (abs(motor_actual_step_vel - motor_encoder_vel)) 
        {
            handle_stall();
        }
        else
        {
            set_velocity(1000);
        }
        
        Serial.println(motor_encoder_vel);
    }
    else if (loop_counter != 0)
    {
        main_loop_triggered = false;
    }
}
