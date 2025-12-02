#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "mpu6050_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

// enums
typedef enum clock_select {
  MPU6050_INTR_8MHz,
  MPU6050_PLL_GYROX,
  MPU6050_PLL_GYROY,
  MPU6050_PLL_GYROZ,
  MPU6050_PLL_EXT_32K,
  MPU6050_PLL_EXT_19MHz,
  MPU6050_STOP = 7,
} mpu6050_clock_select_t;

typedef enum {
  MPU6050_RANGE_2G,  ///< +/- 2g (default value)
  MPU6050_RANGE_4G,  ///< +/- 4g
  MPU6050_RANGE_8G,  ///< +/- 8g
  MPU6050_RANGE_16G, ///< +/- 16g
} mpu6050_accel_range_t;

typedef enum {
  MPU6050_RANGE_250_DEG,  ///< +/- 250 deg/s (default value)
  MPU6050_RANGE_500_DEG,  ///< +/- 500 deg/s
  MPU6050_RANGE_1000_DEG, ///< +/- 1000 deg/s
  MPU6050_RANGE_2000_DEG, ///< +/- 2000 deg/s
} mpu6050_gyro_range_t;

// Structure to hold sensor data
typedef struct {
    float ax, ay, az; // Accelerometer data
    float gx, gy, gz; // Gyroscope data
    float temp;	      // Temperature data
} mpu6050_data_t;

typedef struct {
    mpu6050_clock_select_t clk_sel;
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t gyro_range;
} mpu6050_config_t;

typedef struct {
    int i2c_fd;       // i2c device in Linux
    const char* i2c_device_path;
    mpu6050_data_t* data;
    mpu6050_config_t* cfg;
} mpu6050;

// Core functions
int mpu6050_init(mpu6050* device);
int mpu6050_close(mpu6050* device);

// Data functions
int mpu6050_get_sensors(const mpu6050* device);

#ifdef __cplusplus
}
#endif

#endif
