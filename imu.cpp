#include <math.h>
#include "imu.h"

// complementary filter constant
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
float gyro_bias_x, gyro_bias_y, gyro_bias_z;

// x, y and z angles as derived from the complementary filter
float cf_angle_x;
float cf_angle_y;
float cf_angle_z;

float gyro_angle_x;
float accel_lpf_x;

int32_t prev_cf_angle_x;
int32_t prev_cf_angle_y;
int32_t prev_cf_angle_z;

void imu_init(MPU_9250 *imu)
{
    // init imu
    struct int_param_s int_param;
    imu->begin(int_param);

    // enable only accel and gyro
    imu->setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    
    gyro_full_scale = 2000;
    accel_full_scale = 8;
    imu->setGyroFullScaleRange(gyro_full_scale);
    imu->setAccelFullScaleRange(accel_full_scale);

    gyro_bias_x = 0;
    gyro_bias_y = 0;
    gyro_bias_z = 0;
}

void read_accel_and_gyro(MPU_9250 *imu)
{
    // read raw data
    unsigned long timestamp = 0;
    imu->getAccelReg(imu_raw_data.accel[0], timestamp);
    imu->getGyroReg(imu_raw_data.gyro[0], timestamp);

    // convert data from raw to g for accel and dps for gyro
    imu_data.gyro[0] = (float)imu_raw_data.gyro[0] * gyro_full_scale / 32768 - gyro_bias_x;
    imu_data.gyro[1] = (float)imu_raw_data.gyro[1] * gyro_full_scale / 32768 - gyro_bias_y;
    imu_data.gyro[2] = (float)imu_raw_data.gyro[2] * gyro_full_scale / 32768 - gyro_bias_z;

    imu_data.accel[0] = (float)imu_raw_data.accel[0] * accel_full_scale / 32768;
    imu_data.accel[1] = (float)imu_raw_data.accel[1] * accel_full_scale / 32768;
    imu_data.accel[2] = (float)imu_raw_data.accel[2] * accel_full_scale / 32768;
}

void imu_calibrate_gyro(MPU_9250 *imu)
{
    int i=0;
    
    float bias_sum_x = 0;
    float bias_sum_y = 0;
    float bias_sum_z = 0;

    // take some samples from gyro
    for (i = 0; i < GYRO_CALIBRATION_NSAMPLES; i++)
    {
        read_accel_and_gyro(imu);
        bias_sum_x += imu_data.gyro[0];
        bias_sum_y += imu_data.gyro[1];
        bias_sum_z += imu_data.gyro[2];
        delay(40);
    }

    // get the average from those samples
    gyro_bias_x = bias_sum_x / GYRO_CALIBRATION_NSAMPLES;
    gyro_bias_y = bias_sum_y / GYRO_CALIBRATION_NSAMPLES;
    gyro_bias_z = bias_sum_z / GYRO_CALIBRATION_NSAMPLES;
}

void compl_filter_init()
{   
    cf_angle_x = get_angle_from_accelerometer();
    gyro_angle_x = cf_angle_x;
    cf_angle_y = 0;
    cf_angle_z = 0;
}

float get_angle_from_accelerometer()
{
    return atan2(imu_data.accel[1], imu_data.accel[2]) * 180 / M_PI;
}

void compl_filter_read(int32_t dt_micros)
{   
    // calculate accel angle
    float angle_float_x = get_angle_from_accelerometer();
    //float angle_float_y = atan2(-imu->a_raw.z, imu->a_raw.x) * 180 / M_PI;
    //float angle_float_z = atan2(-imu->a_raw.y, imu->a_raw.x) * 180 / M_PI;
    
    // apply complementary filter 300 us
    gyro_angle_x += imu_data.gyro[0]*dt_micros / 1000000;
    cf_angle_x = CF_CONST*(cf_angle_x + imu_data.gyro[0]*dt_micros/1000000) + (1.0 - CF_CONST)*angle_float_x;
    //cf_angle_y = CF_CONST*(cf_angle_y + unbiased_gyro_y*dt_micros/1000000) + (1.0 - CF_CONST)*accel_angle_y;
    //cf_angle_z = CF_CONST*(cf_angle_z + unbiased_gyro_z*dt_micros/1000000) + (1.0 - CF_CONST)*accel_angle_z;
    //prev_cf_angle_x = cf_angle_x;
    //prev_cf_angle_y = cf_angle_y;
    //prev_cf_angle_z = cf_angle_z;
}
