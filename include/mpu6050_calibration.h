#ifndef MPU6050_CALIBRATION_H
#define MPU6050_CALIBRATION_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <gsl/gsl_math.h>
#include "mpu6050.h"

#define NUM_POINTS_GYRO_CALIBRATION 500
#define CALIBRATION_CYCLE_DELAY_US  1000

#ifdef __cplusplus
extern "C" {
#endif

int mpu6050_calibrate_gyro(mpu6050* device);
int mpu6050_calibrate_accel(mpu6050* device);

#ifdef __cplusplus
}
#endif

#endif
