#include "mpu6050.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    mpu6050_data_t data;
    mpu6050 imu = {-1, "/dev/i2c-1", &data};

    // Initialize MPU6050
    if (mpu6050_init(&imu) < 0) {
        printf("Failed to initialize MPU6050\n");
        return 1;
    }
    
    printf("MPU6050 initialized successfully\n");
    
    // Read data continuously
    while (1) {
	if (mpu6050_read_temp(&imu) == 0) {
		printf("Temperature (C): %f\n", data.temp);
	}
        //if (mpu6050_read_data(&data) == 0) {
        //    printf("Accel: (%d, %d, %d) | Gyro: (%d, %d, %d)\n",
        //           data.ax, data.ay, data.az,
        //           data.gx, data.gy, data.gz);
        //} else {
        //    printf("Failed to read data\n");
        //}
        
        usleep(10000); // 100ms delay
    }
    
    mpu6050_close(&imu);
    return 0;
}
