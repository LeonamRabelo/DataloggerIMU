# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)


# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(DataloggerIMU C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_subdirectory(lib/FatFs_SPI)    

add_executable(DataloggerIMU 
        DataloggerIMU.c
        hw_config.c
        inc/ssd1306.c
    )

pico_set_program_name(DataloggerIMU "DataloggerIMU")
pico_set_program_version(DataloggerIMU "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(DataloggerIMU 1)
pico_enable_stdio_usb(DataloggerIMU 1)

# Add the standard library to the build
target_link_libraries(DataloggerIMU
        pico_stdlib 
        hardware_i2c
        hardware_adc
        FatFs_SPI
        hardware_clocks
        hardware_pwm
        )

# Add the standard include files to the build
target_include_directories(DataloggerIMU PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_SOURCE_DIR}/inc   
)


pico_add_extra_outputs(DataloggerIMU)

