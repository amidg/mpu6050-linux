#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gsl/gsl_fit.h>
#include "mpu6050.h"

#define NUM_POINTS_GYRO_CALIBRATION 500
#define CALIBRATION_CYCLE_DELAY_US  1000

int mpu6050_calibrate_gyro(mpu6050* device) {
    // check if device is valid
    if (!device || !device->data || !device->offset) return -1;

    // check if init was called before:
    // - i2c_fd should be >= 0
    // - i2c_device_path is not NULL
    if (device->i2c_fd < 0 || !device->i2c_device_path) return -1;

    // perform calibration based on the average offset
    device->offset->gx = 0;
    device->offset->gy = 0;
    device->offset->gz = 0;

    for (uint16_t i = 0; i < NUM_POINTS_GYRO_CALIBRATION; ++i) {
        if (mpu6050_get_sensors(device) != 0) return -1;
        device->offset->gx += device->data->gx;
        device->offset->gy += device->data->gy;
        device->offset->gz += device->data->gz;
        usleep(CALIBRATION_CYCLE_DELAY_US); // 1ms
    }

    device->offset->gx /= NUM_POINTS_GYRO_CALIBRATION;
    device->offset->gy /= NUM_POINTS_GYRO_CALIBRATION;
    device->offset->gz /= NUM_POINTS_GYRO_CALIBRATION;

    return 0;
}



int mpu6050_calibrate_accel(mpu6050* device, double* target_y) {
    // check if device is valid
    if (!device || !device->data || !device->offset) return -1;

    // check if init was called before:
    // - i2c_fd should be >= 0
    // - i2c_device_path is not NULL
    if (device->i2c_fd < 0 || !device->i2c_device_path) return -1;

    // get arrays of 500 for all three directions
    double ax[NUM_POINTS_GYRO_CALIBRATION];
    //double ay[NUM_POINTS_GYRO_CALIBRATION];
    //double az[NUM_POINTS_GYRO_CALIBRATION];

    for (uint16_t i = 0; i < NUM_POINTS_GYRO_CALIBRATION; ++i) {
        if (mpu6050_get_sensors(device) != 0) return -1;
        ax[i] = device->data->ax;
        //ay[i] = device->data->ay;
        //az[i] = device->data->az;
        usleep(CALIBRATION_CYCLE_DELAY_US); // 1ms
    }

    // apply gsl_linear_fit
    // Y = C_0 + C_1 * X
    int status = 0;
    double sumsq;
    status = gsl_fit_linear(
        ax, 1,
        target_y, 1,
        NUM_POINTS_GYRO_CALIBRATION,
        &(device->offset->ax_c0),
        &(device->offset->ax_c1),
        &(device->offset->ax_cov00),
        &(device->offset->ax_cov01),
        &(device->offset->ax_cov11),
        &sumsq
    );

    return status;
}

int main(void) {
    mpu6050_data_t data;
    mpu6050_offset_t offset;
    mpu6050_config_t config = {
	MPU6050_PLL_GYROX,
	MPU6050_FSYNC_OUT_ACCELX,
	MPU6050_RANGE_8G,
	MPU6050_RANGE_500_DEG
    };
    mpu6050 imu = {-1, "/dev/i2c-1", &data, &offset, &config};

    // Initialize MPU6050
    if (mpu6050_init(&imu) < 0) {
        printf("Failed to initialize MPU6050\n");
        return 1;
    }
    
    printf("MPU6050 initialized successfully\n");

    // do calibration
    if (mpu6050_calibrate_gyro(&imu) != 0) {
        printf("Failed to calibrate gyroscope MPU6050\n");
        return 1;
    }

    // calibrate accelerometer
    double target_y[500];
    memset(target_y, -1, sizeof(target_y));
    if (mpu6050_calibrate_accel(&imu, target_y) != 0) {
        printf("Failed to calibrate gyroscope MPU6050\n");
        return 1;
    }

    // show offsets
    printf(
        "MPU6050 calibrated successfully:\n"
        "gyro x: %f\n"
        "gyro y: %f\n"
        "gyro z: %f\n"
        "------------\n"
        "accel x: c0 %f, c1 %f\n",
        imu.offset->gx,
        imu.offset->gy,
        imu.offset->gz,
        imu.offset->ax_c0,
        imu.offset->ax_c1
    );

    
    // Read data continuously
    while (1) {
	// read temperature
	if (mpu6050_get_sensors(&imu) == 0) {
		printf("Temperature (C): %f\n", data.temp);
		printf("Accel X / Y / Z (G): %f / %f / %f\n", data.ax, data.ay, data.az);
		printf("Gyro X / Y / Z (rad/s): %f / %f / %f\n", data.gx, data.gy, data.gz);
	}
        
        usleep(100000); // 100ms delay
    }
    
    mpu6050_close(&imu);
    return 0;
}
