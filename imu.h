#ifndef IMU_H
#define IMU_H

extern "C"
{
  #include "avr_i2c.h"
}

#include <mpu9250.h>

// IMU data in units of g for acceleration and degrees per second (dps) for the gyroscope
typedef struct
{
    float accel[3];
    float gyro[3];
} imu_data_t;

// raw IMU data from -32768 to 32767
typedef struct
{
    short accel[3];
    short gyro[3];
} imu_raw_data_t;

// imu data and raw data
extern imu_data_t imu_data;
extern imu_raw_data_t imu_raw_data;

// gyroscope biases
extern float imu_gyro_bias_x, imu_gyro_bias_y, imu_gyro_bias_z;

// x, y and z angles as derived from the complementary filter
extern float imu_cf_angle_x;
extern float imu_cf_angle_y;
extern float imu_cf_angle_z;

// angular velocity around the x axis
extern float imu_gyro_angle_x;

// init the IMU
void imu_init(MPU_9250 *imu);

// read accel and gyro data from IMU in units of g and degrees per second respectively
void imu_read_accel_and_gyro(MPU_9250 *imu);

// find the measurement biases of the gyroscope by averaging over samples
void imu_calibrate_gyro(MPU_9250 *imu);

// init complementary filter
void imu_compl_filter_init();

// get angle from the accelerometer only (fast response, but noisy)
float imu_get_angle_from_accelerometer();

// read gyro and accelerometer, apply complementary filter
// return angle and angular velocities in deg and deg/sec respectively
void imu_compl_filter_read(int32_t dt_micros);

#endif // IMU_H
