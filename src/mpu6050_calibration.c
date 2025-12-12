#include "mpu6050_calibration.h"

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

int mpu6050_calibrate_accel(mpu6050* device) {
    // check if device is valid
    if (!device || !device->data || !device->offset) return -1;

    // check if init was called before:
    // - i2c_fd should be >= 0
    // - i2c_device_path is not NULL
    if (device->i2c_fd < 0 || !device->i2c_device_path) return -1;

    // perform calibration using linear fit
    

    return 0;
}

