cmake_minimum_required(VERSION 3.10)
project(mpu6050 C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# add library
add_library(${PROJECT_NAME} src/mpu6050.c)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Link libraries (if needed)
# For I2C operations, we typically don't need extra linking on Linux
# But we can add compiler flags if needed
target_compile_options(${PROJECT_NAME} PRIVATE 
    -Wall 
    -Wextra 
    -Wpedantic
    -Werror
    -O3
)

# add main.c executable
add_executable(mpu6050_test test/test.c)
target_link_libraries(mpu6050_test PRIVATE ${PROJECT_NAME})
