// the angle in degrees read from the IMU where it is assumed that the robot is perfectly balanced
#define BALANCE_ANGLE -2.5

/* 
 *  the balancing algorithm - consists of two layers of control and some additional tweaks
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
void balance_control(int32_t desired_vel, int32_t desired_turn_rate, int32_t dt_micros);

// reset balance control
void balance_control_reset();

// initialize balance control
void balance_control_init();
