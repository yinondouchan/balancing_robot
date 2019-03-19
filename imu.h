#ifndef COMPL_FILTER_H
#define COMPL_FILTER_H

extern "C"
{
  #include "avr_i2c.h"
}

#include <mpu9250.h>

typedef struct
{
    float accel[3];
    float gyro[3];
} imu_data_t;

typedef struct
{
    short accel[3];
    short gyro[3];
} imu_raw_data_t;

// imu data and raw data
extern imu_data_t imu_data;
extern imu_raw_data_t imu_raw_data;

// gyroscope biases
extern float gyro_bias_x, gyro_bias_y, gyro_bias_z;

// output angles 
extern float cf_angle_x;
extern float cf_angle_y;
extern float cf_angle_z;

// init the IMU
void imu_init(MPU_9250 *imu);

// read accel and gyro data from IMU in units of g and degrees per second respectively
void read_accel_and_gyro(MPU_9250 *imu);

// find the measurement biases of the gyroscope by averaging over samples
void imu_calibrate_gyro(MPU_9250 *imu);

// init
void compl_filter_init();

// read gyro and accelerometer, apply complementary filter
// return angle and angular velocities in millideg and millideg/sec respectively
void compl_filter_read(int32_t dt_micros);

#endif // COMPL_FILTER
