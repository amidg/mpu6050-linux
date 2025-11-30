#include "mpu6050.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>

#define BIT_SET(value, bit)    ((value) |= (1UL << (bit)))
#define BIT_CLEAR(value, bit)  ((value) &= ~(1UL << (bit)))
#define BIT_TOGGLE(value, bit) ((value) ^= (1UL << (bit)))
#define BIT_GET(value, bit)    (((value) >> (bit)) & 1)

// translation unit functions
static int mpu6050_whoami_(const mpu6050* device);
static int mpu6050_reset_(const mpu6050* device);
static int mpu6050_disable_sleep_(const mpu6050* device);
static int mpu6050_set_clock_source_(const mpu6050* device, const mpu6050_clock_select_t* mode);

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
    // page 8 of datasheet -> register 107 will be set to 0x40 = 64
    if (mpu6050_reset_(device) != 0) {
        printf("Failed to reset MPU6050 device\n");
        mpu6050_close(device);
        return -1;
    }

    // set internal clock
    const mpu6050_clock_select_t clk_src = MPU6050_PLL_GYROX;
    if (mpu6050_set_clock_source_(device, &clk_src) != 0) {
	printf("Failed to set clock src at MPU6050 device\n");
        mpu6050_close(device);
        return -1;
    }

    // disable sleep
    if (mpu6050_disable_sleep_(device) != 0) {
        printf("Failed to reset MPU6050 device\n");
        mpu6050_close(device);
        return -1;
    }
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
static int mpu6050_whoami_(const mpu6050* device) {
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

    if (who_am_i != 0x68) return -1;

    return 0;
}

static int mpu6050_set_clock_source_(
	const mpu6050* device, const mpu6050_clock_select_t* mode) {
        uint8_t curr_val = 0;
        uint8_t reg_addr = MPU6050_PWR_MGMT_1;
	uint8_t buffer[2];

	// read current default value
        if (write(device->i2c_fd, &reg_addr, 1) != 1) return -1;
        if (read(device->i2c_fd, &curr_val, 1) != 1) return -1;

	// set last three bits based on the mode
	// set default clock if NULL provided
	curr_val = curr_val | ((!mode) ? MPU6050_INTR_8MHz : *mode);

	// set value of the register
	buffer[0] = reg_addr;
	buffer[1] = curr_val;
        if (write(device->i2c_fd, &buffer, 2) != 2) return -1;
	usleep(100000); // wait 100ms
        if (write(device->i2c_fd, &reg_addr, 1) != 1) return -1;
        if (read(device->i2c_fd, &curr_val, 1) != 1) return -1;
	return 0;
}

static int mpu6050_disable_sleep_(const mpu6050* device) {
	uint8_t curr_val = 0;
        uint8_t reg_addr = MPU6050_PWR_MGMT_1;
        uint8_t buffer[2];

        // read current default value
        if (write(device->i2c_fd, &reg_addr, 1) != 1) return -1;
        if (read(device->i2c_fd, &curr_val, 1) != 1) return -1;

        // set last three bits based on the mode
        // set default clock if NULL provided
	BIT_CLEAR(curr_val, 6);

        // set value of the register
        buffer[0] = reg_addr;
        buffer[1] = curr_val;
        if (write(device->i2c_fd, &buffer, 2) != 2) return -1;
        usleep(100000); // wait 100ms
        if (write(device->i2c_fd, &reg_addr, 1) != 1) return -1;
        if (read(device->i2c_fd, &curr_val, 1) != 1) return -1;
        return 0;
}


static int mpu6050_reset_(const mpu6050* device) {
    uint8_t buffer[2];
    uint8_t curr_val;
    uint8_t reg_addr = 0;
    uint8_t retries = 5;

    // read current value of the MPU6050_PWR_MGMT_1
    reg_addr = MPU6050_PWR_MGMT_1;
    if (write(device->i2c_fd, &reg_addr, 1) != 1) return -1;
    if (read(device->i2c_fd, &curr_val, 1) != 1) return -1;

    uint8_t reset_cmd = curr_val;
    BIT_SET(reset_cmd, 7);
    BIT_CLEAR(reset_cmd, 6);
    buffer[0] = reg_addr;
    buffer[1] = reset_cmd;

    while (curr_val != MPU6050_RESET_OK && retries-- > 0) {
    	if (write(device->i2c_fd, &buffer, 2) != 2) return -1;
    	usleep(100000); // 100ms as required by the datasheet

	// check value
      	if (write(device->i2c_fd, &reg_addr, 1) != 1) return -1;
      	if (read(device->i2c_fd, &curr_val, 1) != 1) return -1;
      	usleep(1000);

	if (retries == 0) {
    	   printf("reset failed: %d\n", curr_val);
	   return -1;
	}
    }

    // signal path reset
    reg_addr = MPU6050_SIGNAL_PATH_RESET;
    buffer[0] = reg_addr;
    buffer[1] = 0x7;
    if (write(device->i2c_fd, &buffer, 2) != 2) return -1;
    usleep(100000); // 100ms as required by the datasheet
    if (write(device->i2c_fd, &reg_addr, 1) != 1) return -1;
    if (read(device->i2c_fd, &curr_val, 1) != 1) {
      printf("reset failed: %d\n", curr_val);
      return -1;
    }

    return 0;
}
