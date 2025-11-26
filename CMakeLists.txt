cmake_minimum_required(VERSION 3.10)
project(mpu6050_example C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Add executable
add_executable(mpu6050_test 
    src/main.c
    src/mpu6050.c
)

# Include directories
target_include_directories(mpu6050_test PRIVATE include)

# Link libraries (if needed)
# For I2C operations, we typically don't need extra linking on Linux
# But we can add compiler flags if needed
target_compile_options(mpu6050_test PRIVATE 
    -Wall 
    -Wextra 
    -Wpedantic
)

# If you want to enable debugging
# target_compile_options(mpu6050_test PRIVATE -g)

# If you want to enable optimization
# target_compile_options(mpu6050_test PRIVATE -O2)
