cmake_minimum_required(VERSION 3.10)
project(mpu6050 C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(COMPILE_OPTIONS
    -Wall 
    -Wextra 
    -Wpedantic
    -Werror
    -O3
)

option(ENABLE_CALIBRATION "Enable calibration features" OFF)

# add library
add_library(${PROJECT_NAME} src/mpu6050.c)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Link libraries (if needed)
# For I2C operations, we typically don't need extra linking on Linux
# But we can add compiler flags if needed
target_compile_options(${PROJECT_NAME} PRIVATE ${COMPILE_OPTIONS})

# add main.c executable
add_executable(mpu6050_test test/test_sensor_read.c)
target_link_libraries(mpu6050_test PRIVATE ${PROJECT_NAME})

# add calibration
if (ENABLE_CALIBRATION)
    # find packages
    find_package(GSL REQUIRED)
    if(NOT GSL_FOUND)
        message(
            FATAL_ERROR
            "GSL not found - please install GSL development package"
        )
    endif()

    # create library
    add_library(${PROJECT_NAME}_calibration src/mpu6050_calibration.c)
    target_include_directories(${PROJECT_NAME}_calibration PUBLIC include)
    target_link_libraries(
        ${PROJECT_NAME}_calibration
        PRIVATE
        ${GSL_LIBRARIES}
    )
    target_compile_options(
        ${PROJECT_NAME}_calibration
        PRIVATE
        ${COMPILE_OPTIONS}
    )

    # create executable for the calibration
    add_executable(mpu6050_calibrate test/test_calibration.c)
    target_link_libraries(
        mpu6050_calibrate
        PRIVATE
        ${PROJECT_NAME}
        ${PROJECT_NAME}_calibration
    )
endif()

