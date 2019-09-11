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

// a nice macro for calculating the sign of a number
#define SIGN(x) (((x) > 0) - ((x) < 0))

// maximum acceleration and deceleration in full steps per second
#define MAX_ACCEL 2000
#define MAX_DECCEL 2000

// the IMU
MPU_9250 imu;

// timer for main loop
volatile int32_t loop_counter;

// true if the robot is upright
bool upright;

// true if the robot is on emergency stop 
bool on_estop;

// true if main loop was already triggered for an iteration
bool main_loop_triggered;

// control velocity after applying acceleration and deceleration constraints
float serial_comm_desired_vel_constrained;

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
    TIMSK1 |= (1 << TOIE1);

    // set fast PWM mode and set prescaler to 8
    TCCR1A += _BV(WGM10);
    TCCR1B |= _BV(CS11) | _BV(WGM12);

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
ISR(TIMER1_OVF_vect)
{
    // count until loop counter reaches the desired value to trigger the main loop
    loop_counter = (loop_counter >= (LOOP_COUNTER_DIVISOR - 1)) ? 0 : loop_counter + 1;
}

// toggle emergency stop
void toggle_estop()
{
    on_estop = !on_estop;
}

// apply acceleration and deceleration constraints to control velocity
void calculate_constrained_control_velocity()
{
    // calculate difference between unconstrained and constraint control velocity
    float vel_diff = serial_comm_desired_vel - serial_comm_desired_vel_constrained;

    // calculate the sign of the above
    int8_t vel_sign = SIGN(vel_diff);

    // check if should accelerate or decelerate
    bool accel = abs(serial_comm_desired_vel) > abs(serial_comm_desired_vel_constrained);

    // set acceleration/decceleration value
    float vel_derivative = accel ? vel_sign * MAX_ACCEL * DT_SECONDS : vel_sign * MAX_DECCEL * DT_SECONDS;

    // if control velocity violates acceleration/deceleration constrains, constrain it. If not - output the same as the control velocity.
    if (abs(vel_derivative) < abs(vel_diff)) serial_comm_desired_vel_constrained += vel_derivative;
    else serial_comm_desired_vel_constrained = serial_comm_desired_vel;
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
    serial_comm_set_estop_callback(toggle_estop);

    loop_counter = 0;

    // initialize motor control
    motor_control_init();

    //initialize encoders
    encoders_init();

    // initialize balancing control
    balance_control_init();

    // emergency stop
    on_estop = false;

    // control velocity after applying acceleration and deceleration constraints
    serial_comm_desired_vel_constrained = 0;

    // prevents main loop being triggered twice per timer interrupt
    main_loop_triggered = false;

    // is the robot upright?
    upright = true;
}

void loop()
{
    // 200Hz loop rate
    if (!main_loop_triggered && (loop_counter == 0))
    {
        main_loop_triggered = true;
        
        // read accel and gyro values
        imu_read_accel_and_gyro(&imu);

        // read tilt angle
        imu_compl_filter_read(DT_MICROS);

        // read encoders
        encoders_read_velocity(MC_LEFT_MOTOR);
        encoders_read_velocity(MC_RIGHT_MOTOR);

        // read data from serial
        serial_comm_read();

        // detect when robot is not upright
        if ((imu_cf_angle_x < -55) || (imu_cf_angle_x > 55)) upright = false;
        if (!upright || on_estop)
        {
            // robot is either not upright or emergency stop is on
            
            // disable the motors
            motor_control_disable_motors();

            // check if robot is upright again
            upright = (imu_cf_angle_x > ((int32_t)BALANCE_ANGLE - 10)) && (imu_cf_angle_x < ((int32_t)BALANCE_ANGLE + 10));

            if (upright) imu_gyro_angle_x = imu_cf_angle_x;

            // reset desired velocity
            serial_comm_desired_vel_constrained = 0;

            // reset motor control variables
            balance_control_reset();
            motor_control_reset();
            return;
        }

        calculate_constrained_control_velocity();

        // filter angular velocity control to avoid those annoying discrete level sounds
        serial_comm_desired_vel_diff_lpf = 0.1 * serial_comm_desired_vel_diff + 0.9 * serial_comm_desired_vel_diff_lpf;

        // main balancing algorithm
        balance_control(serial_comm_desired_vel_constrained, - serial_comm_desired_vel_diff_lpf, DT_MICROS);
    }
    else if (loop_counter != 0)
    {
        main_loop_triggered = false;
    }
}
