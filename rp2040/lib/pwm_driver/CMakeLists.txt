add_library(pwm_driver pwm_driver.cpp pwm_driver.h)
target_link_libraries(pwm_driver pico_stdlib hardware_pwm hardware_clocks)
target_include_directories(pwm_driver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")


