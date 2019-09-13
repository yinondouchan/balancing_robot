// the angle in degrees read from the IMU where it is assumed that the robot is perfectly balanced
#define BALANCE_ANGLE -2.5

// a nice macro for calculating the sign of a number
#define SIGN(x) (((x) > 0) - ((x) < 0))

// maximum acceleration and deceleration in full steps per second
#define MAX_ACCEL 2000
#define MAX_DECCEL 2000

// toggle emergency stop
void balance_control_toggle_estop();

// apply acceleration and deceleration constraints to control velocity
void balance_control_calculate_constrained_control_velocity();

// this is where all the balance control is done.
// In this implementation, the robot either lays down or keeps itself upright
void balance_control();

// lay down
void balance_control_lay_lown();

/* 
 *  the balancing algorithm to keep the robot upright - consists of two layers of control and some additional tweaks
 *  
 *  bottom layer: controls motor velocity as a function of the tilt angle and tilt angular velocity.
 *  It differs from a PID controller by the output being an integral of a linear combination of the tilt angle and the tilt angular velocity.
 *  
 *  top layer: controls tilt angle as a function of the desired velocity.
 *  This layer is implemented as a PID controller with the control output being the tilt angle 
 *  and the error signal being the error between the desired motor velocity and the actual motor velocity
 *  
 *  additional tweaks:
 *  - an additional D element on the velocity error is added to the output of the aforementioned two layer controller. 
 *  This nearly abolishes wobbliness of the robot at higher speeds
 *  - top layer: change I component only when control tells the robot to stop (desired vel and turn rate are 0). 
 *  This removes unnecessary wobbliness in lower speeds by decreasing I component windup.
 */
void balance_control_keep_upright(int32_t desired_vel, int32_t desired_turn_rate, int32_t dt_micros);

// reset balance control
void balance_control_reset();

// initialize balance control
void balance_control_init();
