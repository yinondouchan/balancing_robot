// clock speed
#define CLOCK_SPEED_HZ 16000000

// various motor drivers
#define MOTOR_DRIVER_A4988 0
#define MOTOR_DRIVER_MP6500 1
#define MOTOR_DRIVER_TB67S249 2
#define MOTOR_DRIVER_DRV8825 3

// currently used stepper motor driver
#define MOTOR_DRIVER MOTOR_DRIVER_DRV8825

// main loop time interval in microseconds
#define DT_MICROS 5000

// main loop time interval in seconds
#define DT_SECONDS 0.005

// number of full steps per motor revolution
#define FULL_STEPS_PER_REV 200

// tunable parameters
typedef struct
{
    float angle_pid_p;
    float angle_pid_ang_vel_p;
    float vel_pid_p;
    float vel_pid_i;
    float vel_pid_d;
    float vel_lpf_tc;
    int32_t balance_vel_limit;
    float vel_pid_d2;
    float current_limit;
} parameters_t;

// tunable parameters tags
enum parameter_tlv_tag {
                        ANGLE_PID_P,                // inner PID loop angle proportional component
                        ANGLE_PID_ANG_VEL_P,        // inner PID loop angular velocity proportional component
                        VEL_PID_P,                  // outer PID loop velocity proportional
                        VEL_PID_I,                  // outer PID loop velocity integral
                        VEL_PID_D,                  // outer PID loop velocity derivative
                        VEL_LPF_TC,                 // velocity low-pass filter time constant [microseconds]
                        BALANCE_VEL_LIMIT,          // balancing algorithm velocity limit [full-steps per second]
                        VEL_PID_D2,
                        CURRENT_LIMIT,
                        };

// tunable parameters
extern parameters_t parameters;

// set default tunable parameters
void parameters_set_default();

// set a tunable parameter
void parameters_set_parameter(uint8_t tag, uint8_t len, uint8_t *value);
