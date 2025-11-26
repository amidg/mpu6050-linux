#include "mpu6050.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>

// translation unit functions
static int mpu6050_whoami_(mpu6050* device);
static int mpu6050_reset_(mpu6050* device);

// API functions
int mpu6050_close(mpu6050* device) {
    if (device->i2c_fd >= 0) {
        close(device->i2c_fd);
        device->i2c_fd = -1;
        return 0;
    }
    return -1;
}

int mpu6050_init(mpu6050* device) {
    // Open I2C device
    device->i2c_fd = open(device->i2c_device_path, O_RDWR);
    if (device->i2c_fd < 0) {
        printf("Failed to open I2C device\n");
        mpu6050_close(device);
        return -1;
    }
    
    // Set I2C address
    if (ioctl(device->i2c_fd, I2C_SLAVE, MPU6050_I2CADDR_DEFAULT) < 0) {
        printf("Failed to set I2C address\n");
        mpu6050_close(device);
        return -1;
    }

    // who am i
    if (mpu6050_whoami_(device) != 0) {
        printf("Failed to set WHO_AM_I\n");
        mpu6050_close(device);
        return -1;
    }

    // reset
    if (mpu6050_reset_(device) != 0) {
        printf("Failed to reset MPU6050 device\n");
        mpu6050_close(device);
        return -1;
    }
    return 0;
}

static int mpu6050_reset_(mpu6050* device) {
    // set power management register
    //if (write(device->i2c_fd, ))
    (void)device;
    return 0;
}

//int mpu6050_read_data(mpu6050_data_t* data) {
//    if (i2c_fd < 0) {
//        return -1;
//    }
//    
//    // Read accelerometer data (6 bytes)
//    unsigned char accel_data[6];
//    if (read(i2c_fd, accel_data, 6) != 6) {
//        perror("Failed to read accelerometer data");
//        return -1;
//    }
//    
//    // Read gyroscope data (6 bytes)
//    unsigned char gyro_data[6];
//    if (read(i2c_fd, gyro_data, 6) != 6) {
//        perror("Failed to read gyroscope data");
//        return -1;
//    }
//    
//    // Convert bytes to signed 16-bit integers
//    data->ax = (accel_data[0] << 8) | accel_data[1];
//    data->ay = (accel_data[2] << 8) | accel_data[3];
//    data->az = (accel_data[4] << 8) | accel_data[5];
//    
//    data->gx = (gyro_data[0] << 8) | gyro_data[1];
//    data->gy = (gyro_data[2] << 8) | gyro_data[3];
//    data->gz = (gyro_data[4] << 8) | gyro_data[5];
//    
//    return 0;
//}


// Static functions
static int mpu6050_whoami_(mpu6050* device) {
    // Verify device by reading WHO_AM_I register
    unsigned char who_am_i;
    unsigned char reg_addr = MPU6050_WHO_AM_I;
    if (write(device->i2c_fd, &reg_addr, 1) != 1) {
        printf("Failed to set WHO_AM_I address on the i2c bus\n");
        return -1;
    }

    if (read(device->i2c_fd, &who_am_i, 1) != 1) {
        printf("Failed to read WHO_AM_I register\n");
        return -1;
    }
    
    return 0;
}
