#include "mpu6050.h"
#include <stdio.h>
#include <unistd.h>

int main(void) {
    mpu6050_data_t data;
    mpu6050_config_t config = {MPU6050_PLL_GYROX, MPU6050_RANGE_8G, MPU6050_RANGE_500_DEG};
    mpu6050 imu = {-1, "/dev/i2c-1", &data, &config};

    // Initialize MPU6050
    if (mpu6050_init(&imu) < 0) {
        printf("Failed to initialize MPU6050\n");
        return 1;
    }
    
    printf("MPU6050 initialized successfully\n");
    
    // Read data continuously
    while (1) {
	// read temperature
	if (mpu6050_get_sensors(&imu) == 0) {
		printf("Temperature (C): %f\n", data.temp);
		printf("Accel X / Y / Z (): %d / %d / %d\n", data.ax, data.ay, data.az);
		printf("Gyro X / Y / Z (): %d / %d / %d\n", data.gx, data.gy, data.gz);
	}
        
        usleep(100000); // 100ms delay
    }
    
    mpu6050_close(&imu);
    return 0;
}
