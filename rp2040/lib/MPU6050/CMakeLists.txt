add_library(mpu6050 mpu6050.cpp mpu6050.h mpu6050_dmp.h)
target_link_libraries(mpu6050 pico_stdlib hardware_i2c hardware_pio i2c_driver)
target_include_directories(mpu6050 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")


