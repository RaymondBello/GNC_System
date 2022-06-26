cmake_minimum_required(VERSION 3.19)

# initialize the SDK based on PICO_SDK_PATH
include(pico_sdk_import.cmake)

# project name
project(RP2040_UAV C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


# initialize the Pico SDK
pico_sdk_init()


add_executable(RP2040_UAV
    src/main.cpp
)

FILE(GLOB FreeRTOS_src FreeRTOS-Kernel/*.c)

add_library( FreeRTOS STATIC
	${FreeRTOS_src}
	FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
	FreeRTOS-Kernel/portable/MemMang/heap_4.c
)

target_include_directories( FreeRTOS PUBLIC
    FreeRTOS-Kernel/include
    FreeRTOS-Kernel/portable/GCC/ARM_CM0
    include/
)

add_subdirectory(lib/i2c_driver)
add_subdirectory(lib/MPU6050)
add_subdirectory(lib/pwm_driver)


target_link_libraries( RP2040_UAV 
    pico_stdlib 
    hardware_gpio 
    hardware_i2c 
    hardware_pio
    hardware_pwm
    FreeRTOS
    i2c_driver
    mpu6050
    pwm_driver
)

add_compile_options(-Wall
    # -Wno-format                     # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
    # -Wno-unused-function            # we have some, for the function's that aren't called
    # -Wno-maybe-uninitialized        # 
    # -Wno-dev
)

# enable usb output, disable uart output
pico_enable_stdio_usb(RP2040_UAV 1)
pico_enable_stdio_uart(RP2040_UAV 0)
	
pico_add_extra_outputs(RP2040_UAV)