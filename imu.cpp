#include <math.h>
#include "imu.h"

// complementary filter constant
// currently set to a very high value due to the balancing algorithm being able to compensate on the gyro's bias
#define CF_CONST 0.999

// number of averaging samples for gyro calibration
#define GYRO_CALIBRATION_NSAMPLES 200

// hold IMU data and raw data
imu_data_t imu_data;
imu_raw_data_t imu_raw_data;

// gyro and accel full scale ranges as init in the IMU
uint16_t gyro_full_scale;
uint8_t accel_full_scale;

// gyro biases in x, y and z axes
float imu_gyro_bias_x, imu_gyro_bias_y, imu_gyro_bias_z;

// x, y and z angles as derived from the complementary filter
float imu_cf_angle_x;
float imu_cf_angle_y;
float imu_cf_angle_z;

// angle obtained by integrating the gyro values
float imu_gyro_angle_x;

// init the IMU
void imu_init(MPU_9250 *imu)
{
    // init imu
    struct int_param_s int_param;
    imu->begin(int_param);

    // enable only accel and gyro
    imu->setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

    // set gyro and accelerometer full scale values
    gyro_full_scale = 2000;
    accel_full_scale = 8;
    imu->setGyroFullScaleRange(gyro_full_scale);
    imu->setAccelFullScaleRange(accel_full_scale);

    // init gyro biases
    imu_gyro_bias_x = 0;
    imu_gyro_bias_y = 0;
    imu_gyro_bias_z = 0;

    // calibrate gyro
    imu_calibrate_gyro(imu);

    // init complementary filter
    imu_compl_filter_init();
}

void imu_read_accel_and_gyro(MPU_9250 *imu)
{
    // read raw data
    unsigned long timestamp = 0;
    imu->getAccelReg(imu_raw_data.accel[0], timestamp);
    imu->getGyroReg(imu_raw_data.gyro[0], timestamp);

    // convert gyro data from raw to dps
    imu_data.gyro[0] = (float)imu_raw_data.gyro[0] * gyro_full_scale / 32768 - imu_gyro_bias_x;
    imu_data.gyro[1] = (float)imu_raw_data.gyro[1] * gyro_full_scale / 32768 - imu_gyro_bias_y;
    imu_data.gyro[2] = (float)imu_raw_data.gyro[2] * gyro_full_scale / 32768 - imu_gyro_bias_z;

    // convert accelerometer data from raw to g
    imu_data.accel[0] = (float)imu_raw_data.accel[0] * accel_full_scale / 32768;
    imu_data.accel[1] = (float)imu_raw_data.accel[1] * accel_full_scale / 32768;
    imu_data.accel[2] = (float)imu_raw_data.accel[2] * accel_full_scale / 32768;
}

// find the measurement biases of the gyroscope by averaging over samples
void imu_calibrate_gyro(MPU_9250 *imu)
{
    int i=0;
    float bias_sum_x = 0;
    float bias_sum_y = 0;
    float bias_sum_z = 0;

    // take some samples from gyro
    for (i = 0; i < GYRO_CALIBRATION_NSAMPLES; i++)
    {
        imu_read_accel_and_gyro(imu);
        bias_sum_x += imu_data.gyro[0];
        bias_sum_y += imu_data.gyro[1];
        bias_sum_z += imu_data.gyro[2];
        delay(40);
    }

    // get the average from those samples
    imu_gyro_bias_x = bias_sum_x / GYRO_CALIBRATION_NSAMPLES;
    imu_gyro_bias_y = bias_sum_y / GYRO_CALIBRATION_NSAMPLES;
    imu_gyro_bias_z = bias_sum_z / GYRO_CALIBRATION_NSAMPLES;
}

// init complementary filter
void imu_compl_filter_init()
{   
    // initialize angle to the one read from the accelerometer - this is a fast, though noisy way to initialize the angle
    imu_cf_angle_x = imu_get_angle_from_accelerometer();
    imu_gyro_angle_x = imu_cf_angle_x;
}

// get angle from the accelerometer only (fast response, but noisy)
float imu_get_angle_from_accelerometer()
{
    // an arc-tan a day reads the angle away
    return atan2(imu_data.accel[1], imu_data.accel[2]) * 180 / M_PI;
}

// read gyro and accelerometer, apply complementary filter
// return angle and angular velocities in deg and deg/sec respectively
void imu_compl_filter_read(int32_t dt_micros)
{  
    // only read x value since there is no need in other values (yet)
   
    // calculate accel angle
    float angle_float_x = imu_get_angle_from_accelerometer();
    
    // apply complementary filter 300 us
    imu_gyro_angle_x += imu_data.gyro[0]*dt_micros / 1000000;
    imu_cf_angle_x = CF_CONST*(imu_cf_angle_x + imu_data.gyro[0]*dt_micros/1000000) + (1.0 - CF_CONST)*angle_float_x;
}
