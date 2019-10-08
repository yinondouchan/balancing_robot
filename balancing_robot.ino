/*
 * Main file for the balancing robot.
 */

#include <Arduino.h>

extern "C"
{
  #include "avr_i2c.h"
}

#include <mpu9250.h>
#include "imu.h"
#include "serial_comm.h"
#include "motor_control.h"
#include "balance_control.h"
#include "encoders.h"
#include "parameters.h"

// how many timer interrups to count until main loop is triggered
#define LOOP_COUNTER_DIVISOR 40

// the IMU
MPU_9250 imu;

// timer for main loop
volatile int32_t loop_counter;

// true if main loop was already triggered for an iteration
bool main_loop_triggered;

// setup the robot's timers
// timer 0 is used for controlling right stepper motor
// timer 1 is used for controlling main loop
// timer 2 is used for controlling left stepper motor
void setup_timers()
{
    // clear all interrupts
    cli();

    // initialize Timer0 for controlling right stepper motor
    TCCR0A = 0;    // set entire TCCR0A register to 0
    TCCR0B = 0;    // set entire TCCR0B register to 0 
                   // (as we do not know the initial  values)

    // enable output compare interrupt for timer 0
    TIMSK0 |= (1 << OCIE0A);

    // set initial output compare register value for timer 0
    OCR0A = 255;

    // set CTC mode (clear timer on compare-match) and set clock prescaler to 8
    TCCR0A |= _BV(WGM01);
    TCCR0B |= _BV(CS01);

    // initialize Timer1 for main loop - a 7812.5 Hz interrupt rate
    TCCR1A = 0;    // set entire TCCR1A register to 0
    TCCR1B = 0;    // set entire TCCR1B register to 0 
                   // (as we do not know the initial  values)

    // enable overflow interrupt for timer 1
    TIMSK1 |= (1 << OCIE1A);

    // set prescaler to 8
    TCCR1B |= _BV(CS11);
    
    // set CTC mode
    TCCR1B |= _BV(WGM12);

    // trigger output compare match once in 250 compare match interrupts
    // with the above prescaler we get an interrup frequency of 16000000 / 8 / 250 = 8000 Hz
    OCR1A = 249;

    // initialize Timer2 for controlling left stepper motor
    TCCR2A = 0;    // set entire TCCR2A register to 0
    TCCR2B = 0;    // set entire TCCR2B register to 0 
                   // (as we do not know the initial  values)

    // set initial output compare register value for timer 2
    OCR2A = 255;

    // enable output compare interrupt for timer 0
    TIMSK2 |= (1 << OCIE2A);

    // CTC mode and clock prescaler 8 (same as timer 0)
    TCCR2B |= _BV(CS21);
    TCCR2A |= _BV(WGM21);
    
    // enable global interrupts:
    sei();
}

// overflow interrupt for timer 1 (main loop timer)
ISR(TIMER1_COMPA_vect)
{
    // count until loop counter reaches the desired value to trigger the main loop
    loop_counter = (loop_counter >= (LOOP_COUNTER_DIVISOR - 1)) ? 0 : loop_counter + 1;
}

// setup
void setup()
{
    // set up serial
    Serial.begin(9600);
  
    // set up the timers
    setup_timers();
    
    // set the default tunable parameters
    parameters_set_default();
    
    // init imu
    Serial.println("Initializing IMU");
    imu_init(&imu);
    Serial.println("Done initializing IMU");

    // init serial comm
    serial_comm_init();
    serial_comm_set_param_callback(parameters_set_parameter);
    serial_comm_set_estop_callback(balance_control_on_estop);

    loop_counter = 0;

    // initialize motor control
    motor_control_init();

    //initialize encoders
    encoders_init();

    // initialize balancing control
    balance_control_init();

    // prevents main loop being triggered twice per timer interrupt
    main_loop_triggered = false;
}

void read_inputs()
{
    // read accel and gyro values
    imu_read_accel_and_gyro(&imu);

    // read tilt angle
    imu_compl_filter_read(DT_MICROS);

    // read encoders if stall detection is enabled
#if ENABLE_STALL_DETECTION
    encoders_read_velocity(MC_LEFT_MOTOR);
    encoders_read_velocity(MC_RIGHT_MOTOR);
#endif

    // read data from serial
    serial_comm_read();
}

void loop()
{
    // 200Hz loop rate
    if (!main_loop_triggered && (loop_counter == 0))
    {
        main_loop_triggered = true;

        // read all inputs
        read_inputs();

        // control balance
        balance_control();
    }
    else if (loop_counter != 0)
    {
        main_loop_triggered = false;
    }
}
