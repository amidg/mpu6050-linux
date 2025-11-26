#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "mpu6050_reg.h"

// Structure to hold sensor data
typedef struct {
    int16_t ax, ay, az; // Accelerometer data
    int16_t gx, gy, gz; // Gyroscope data
    float   temp;
} mpu6050_data_t;

typedef struct {
    int i2c_fd;       // i2c device in Linux
    const char* i2c_device_path;
    mpu6050_data_t* data;
} mpu6050;

// Init functions
int mpu6050_init(mpu6050* device);
//int mpu6050_reset(mpu6050* device);

// Data functions
//int mpu6050_read_temp(float* temp);
//int mpu6050_read_accel(int16_t* accel_arr);
//int mpu6050_read_gyro(int16_t* gyro_arr);
//int mpu6050_read_data(mpu6050_data_t* data);
int mpu6050_close(mpu6050* device);

#endif
