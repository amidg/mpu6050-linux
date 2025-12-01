#ifndef MPU6050_REG_H
#define MPU6050_REG_H

// Register values
// Copied from the Adafruit_MPU6050.h
// https://github.com/adafruit/Adafruit_MPU6050/blob/master/Adafruit_MPU6050.h
#define MPU6050_I2CADDR_DEFAULT 	0x68 ///< MPU6050 default i2c address w/ AD0 high
#define MPU6050_WHO_AM_I 		0x75 ///< Divice ID register
#define MPU6050_DEVICE_ID		0x68 ///< The correct MPU6050_WHO_AM_I value
#define MPU6050_SELF_TEST_X		0x0D ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Y		0x0E ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_Z		0x0F ///< Self test factory calibrated values register
#define MPU6050_SELF_TEST_A		0x10 ///< Self test factory calibrated values register
#define MPU6050_SMPLRT_DIV 		0x19 ///< sample rate divisor register
#define MPU6050_CONFIG 			0x1A ///< General configuration register
#define MPU6050_GYRO_CONFIG 		0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG 		0x1C ///< Accelerometer specific configration register
#define MPU6050_INT_PIN_CONFIG 		0x37 ///< Interrupt pin configuration register
#define MPU6050_INT_ENABLE 		0x38 ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS 		0x3A ///< Interrupt status register
#define MPU6050_SIGNAL_PATH_RESET 	0x68 ///< Signal path reset register
#define MPU6050_USER_CTRL 		0x6A ///< FIFO and I2C Master control register
#define MPU6050_PWR_MGMT_1 		0x6B ///< Primary power/sleep control register
#define MPU6050_PWR_MGMT_2		0x6C ///< Secondary power/sleep control register
#define MPU6050_TEMP_H			0x41 ///< Temperature data high byte register
#define MPU6050_TEMP_L			0x42 ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT		0x3B ///< base address for all sensor/accel data reads
#define MPU6050_TEMP_OUT		0x41 ///< base address for temperature data reads
#define MPU6050_GYRO_OUT		0x43 ///< base address for gyro data reads
#define MPU6050_MOT_THR			0x1F ///< Motion detection threshold bits [7:0]
#define MPU6050_MOT_DUR 		0x20 ///< Duration counter threshold for motion int.
					     ///  1 kHz rate, LSB = 1 ms
// constants
#define MPU6050_RESET_OK     		0x40 // default value as outlined on page 41
#define FS_SEL_RESET_VALUE		0xE7 // bit mask to reset gyro and accel

#endif
