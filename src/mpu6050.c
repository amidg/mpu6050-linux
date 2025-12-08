#include "mpu6050.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define BIT_SET(value, bit)    ((value) |= (1UL << (bit)))
#define BIT_CLEAR(value, bit)  ((value) &= ~(1UL << (bit)))
#define BIT_TOGGLE(value, bit) ((value) ^= (1UL << (bit)))
#define BIT_GET(value, bit)    (((value) >> (bit)) & 1)

// basic functions
static int read_i2c_(
	const int fd, const uint8_t* addr, const uint8_t size_bytes, uint8_t* result) {
	// check if FD is valid
	if (fd < 0) return -1;
	if (!addr) return -1;
	if (!result) return -1;

	// write the register addr then read from it
	if (write(fd, addr, 1) != 1) return -1;
	if (read(fd, result, size_bytes) != size_bytes) return -1;
	return 0;
}

static int write_i2c_(
	const int fd, const uint8_t* buffer, const uint8_t size_bytes) {
	// assumptions:
	// - first member of the buffer is the address, not the buffer value itself
	if (fd < 0) return -1;
	if (!buffer) return -1;

	// write buffer
	if (write(fd, buffer, size_bytes) != size_bytes) return -1;
	return 0;
}

// translation unit functions
static int mpu6050_whoami_(const mpu6050* device);
static int mpu6050_reset_(const mpu6050* device);
static int mpu6050_disable_sleep_(const mpu6050* device);
static int mpu6050_set_clock_source_(const mpu6050* device);
static int mpu6050_set_fsync_(const mpu6050* device);
static int mpu6050_set_gyro_range_(const mpu6050* device);
static int mpu6050_set_accel_range_(const mpu6050* device);

// API functions
int mpu6050_close(mpu6050* device) {
    if (!device) return -1;
    if (device->i2c_fd >= 0) {
        close(device->i2c_fd);
        device->i2c_fd = -1;
        return 0;
    }
    return -1;
}

int mpu6050_init(mpu6050* device) {
    // check
    if (!device) {
	    printf("Requested NULL device");
	    return -1;
    }

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
    if (mpu6050_set_clock_source_(device) != 0) {
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

    // set fsync
    if (mpu6050_set_fsync_(device) != 0) {
	printf("Failed to set fsync at MPU6050 device\n");
        mpu6050_close(device);
        return -1;
    }

    // set gyro range
    if (mpu6050_set_gyro_range_(device) != 0) {
	printf("Failed to set gyro range at MPU6050 device\n");
        mpu6050_close(device);
        return -1;
    }

    // set gyro range
    if (mpu6050_set_accel_range_(device) != 0) {
	printf("Failed to set accel range at MPU6050 device\n");
        mpu6050_close(device);
        return -1;
    }

    return 0;
}

// read 14-bit buffer from the 
int mpu6050_get_sensors(const mpu6050* device) {
 	// checks
	if (!device || !device->data) return -1;
	static float accel_scale = 1;
	if (device->cfg != NULL) {
		switch (device->cfg->accel_range) {
		case MPU6050_RANGE_2G:
			accel_scale = 16384.0f;
			break;
		case MPU6050_RANGE_4G:
			accel_scale = 8192.0f;
			break;
		case MPU6050_RANGE_8G:
			accel_scale = 4096.0f;
			break;
		case MPU6050_RANGE_16G:
			accel_scale = 2048.0f;
			break;
		}
	}

	static float gyro_scale = 1;
	if (device->cfg != NULL) {
		switch (device->cfg->gyro_range) {
		case MPU6050_RANGE_250_DEG:
			gyro_scale = 131.0f;
			break;
		case MPU6050_RANGE_500_DEG:
			gyro_scale = 65.5f;
			break;
		case MPU6050_RANGE_1000_DEG:
			gyro_scale = 32.8f;
			break;
		case MPU6050_RANGE_2000_DEG:
			gyro_scale = 16.4f;
			break;
		}
	}

	// buffer consists of:
	// - 6x uint8_t ACCEL values (HIGH and LOW)
	// - 2x uint8_t TEMP values (HIGH and LOW)
	// - 6x uint8_t GYRO values (HIGH and LOW)
	// MPU6050_ACCEL_OUT (0x3B) is starting register
	const uint8_t addr = MPU6050_ACCEL_OUT;
	uint8_t buffer[14];

        if (read_i2c_(device->i2c_fd, &addr, 14, buffer) != 0) return -1;

	// read accel
	device->data->ax = ((int16_t)((buffer[0] << 8) | buffer[1])) / accel_scale;
	device->data->ay = ((int16_t)((buffer[2] << 8) | buffer[3])) / accel_scale;
	device->data->az = ((int16_t)((buffer[4] << 8) | buffer[5])) / accel_scale;
	
	// read temp
	device->data->temp = (int16_t)((buffer[6] << 8) | buffer[7]) / 340.0f + 36.53f;

	// read gyro
	device->data->gx = (M_PI * ((int16_t)((buffer[8] << 8) | buffer[9])) / gyro_scale) / 180.0;
	device->data->gy = (M_PI * ((int16_t)((buffer[10] << 8) | buffer[11])) / gyro_scale) / 180.0;
	device->data->gz = (M_PI * ((int16_t)((buffer[12] << 8) | buffer[13])) / gyro_scale) / 180.0;

	return 0;
}

// Static functions
static int mpu6050_whoami_(const mpu6050* device) {
    // Verify device by reading WHO_AM_I register
    uint8_t who_am_i;
    const uint8_t reg_addr = MPU6050_WHO_AM_I;
    // const int* fd, const uint8_t* addr, const uint8_t num_bytes, uint8_t* result) {
    if (read_i2c_(device->i2c_fd, &reg_addr, 1, &who_am_i) != 0) {
        printf("Failed to set WHO_AM_I address on the i2c bus\n");
    }

    if (who_am_i != 0x68) return -1;

    return 0;
}

static int mpu6050_reset_(const mpu6050* device) {
    uint8_t buffer[2] = {MPU6050_PWR_MGMT_1};
    uint8_t retries = 5;

    // read current value of the MPU6050_PWR_MGMT_1
    if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;

    // set command value to do reset
    BIT_SET(buffer[1], 7);
    const uint8_t cmd = buffer[1];

    while (buffer[1] != MPU6050_RESET_OK && retries-- > 0) {
	buffer[1] = cmd;
	if (write_i2c_(device->i2c_fd, buffer, sizeof(buffer)) != 0) return -1;
    	usleep(100000); // 100ms as required by the datasheet

	// check value
        if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;

	if (retries == 0) {
    	   printf("reset failed: %d\n", buffer[1]);
	   return -1;
	}
    }

    // signal path reset (write only)
    // page 37
    buffer[0] = MPU6050_SIGNAL_PATH_RESET;
    buffer[1] = 0x7;
    if (write_i2c_(device->i2c_fd, buffer, sizeof(buffer)) != 0) return -1;
    usleep(100000); // 100ms as required by the datasheet

    return 0;
}

// translation units
static int mpu6050_set_clock_source_(const mpu6050* device) {
	// checks
	if (!device) return -1;

	uint8_t buffer[2] = {MPU6050_PWR_MGMT_1};

	// read current default value
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;

	// set last three bits based on the mode
	// set default clock if NULL provided
	const mpu6050_clock_select_t mode =
	    (!(device->cfg)) ? MPU6050_INTR_8MHz : device->cfg->clk_sel;
	buffer[1] |= mode;

	// set value of the register
	if (write_i2c_(device->i2c_fd, buffer, sizeof(buffer)) != 0) return -1;
	usleep(100000); // wait 100ms

	// check the written value
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;
	if ((buffer[1] & mode) != mode) return -1;
	return 0;
}

static int mpu6050_set_fsync_(const mpu6050* device) {
	// checks
	if (!device) return -1;

	uint8_t buffer[2] = {MPU6050_CONFIG};

	// read current default value
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;

	// set last three bits based on the mode
	// set default clock if NULL provided
	const mpu6050_fsync_out_t mode = 
	    (!(device->cfg)) ? MPU6050_FSYNC_OUT_DISABLED : device->cfg->fsync_sel;
	const uint8_t ext_sync_set = ((mode & 0x07) << 3);
	buffer[1] |= ext_sync_set;

	// set value of the register
	if (write_i2c_(device->i2c_fd, buffer, sizeof(buffer)) != 0) return -1;
	usleep(100000); // wait 100ms

	// check the written value
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;
	if ((buffer[1] & ext_sync_set) != ext_sync_set) return -1;
	return 0;
}

static int mpu6050_disable_sleep_(const mpu6050* device) {
        uint8_t buffer[2] = {MPU6050_PWR_MGMT_1};

	if (!device) return -1;

	// check current values
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;

        // set last three bits based on the mode
        // set default clock if NULL provided
	BIT_CLEAR(buffer[1], 6);

        // set value of the register
	if (write_i2c_(device->i2c_fd, buffer, sizeof(buffer)) != 0) return -1;
        usleep(100000); // wait 100ms
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;
	if ((buffer[1] & MPU6050_RESET_OK) != 0) return -1;
	return 0;
}

static int mpu6050_set_gyro_range_(const mpu6050* device) {
	// checks
	if (!device) return -1;

	uint8_t buffer[2] = {MPU6050_GYRO_CONFIG};

	// read current default value
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;

	// set last three bits based on the mode
	// set default if NULL provided
	mpu6050_gyro_range_t mode =
	    (!(device->cfg)) ? MPU6050_RANGE_250_DEG : device->cfg->gyro_range;
	buffer[1] &= FS_SEL_RESET_VALUE; // clear FS_SEL bits
	const uint8_t mode_shifted = ((mode & 0x03) << 3);
	buffer[1] |= mode_shifted;

	// set value of the register
	if (write_i2c_(device->i2c_fd, buffer, sizeof(buffer)) != 0) return -1;
	usleep(100000); // wait 100ms

	// check the written value
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;
	if ((buffer[1] & mode_shifted) != mode_shifted) return -1;
	return 0;
}

static int mpu6050_set_accel_range_(const mpu6050* device) {
	// checks
	if (!device) return -1;

	uint8_t buffer[2] = {MPU6050_ACCEL_CONFIG};

	// read current default value
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;

	// set last three bits based on the mode
	// set default if NULL provided
	const mpu6050_accel_range_t mode =
	    (!(device->cfg)) ? MPU6050_RANGE_2G : device->cfg->accel_range;
	buffer[1] &= FS_SEL_RESET_VALUE; // clear FS_SEL bits
	const uint8_t mode_shifted = ((mode & 0x03) << 3);
	buffer[1] |= mode_shifted;

	// set value of the register
	if (write_i2c_(device->i2c_fd, buffer, sizeof(buffer)) != 0) return -1;
	usleep(100000); // wait 100ms

	// check the written value
	if (read_i2c_(device->i2c_fd, buffer, 1, &buffer[1]) != 0) return -1;
	if ((buffer[1] & mode_shifted) != mode_shifted) return -1;
	return 0;
}
