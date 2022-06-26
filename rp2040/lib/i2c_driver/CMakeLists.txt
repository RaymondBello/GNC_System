add_library(i2c_driver I2Cdev.cpp I2Cdev.h)
target_link_libraries(i2c_driver pico_stdlib hardware_i2c pico_double)
target_include_directories(i2c_driver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")


