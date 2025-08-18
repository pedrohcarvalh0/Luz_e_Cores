# Generated Cmake Pico project file
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(Luz_e_Cores C CXX ASM)
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Luz_e_Cores Luz_e_Cores.c lib/bh1750_light_sensor.c 
        lib/ssd1306.c)

pico_generate_pio_header(Luz_e_Cores ${CMAKE_CURRENT_LIST_DIR}/lib/ws2812.pio)

pico_set_program_name(Luz_e_Cores "Luz_e_Cores")
pico_set_program_version(Luz_e_Cores "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Luz_e_Cores 0)
pico_enable_stdio_usb(Luz_e_Cores 0)

# Add the standard library to the build
target_link_libraries(Luz_e_Cores
        pico_stdlib
        hardware_pwm
        hardware_pio
        hardware_i2c)

# Add the standard include files to the build
target_include_directories(Luz_e_Cores PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/lib
)

# Add any user requested libraries
target_link_libraries(Luz_e_Cores 
        
        )

pico_add_extra_outputs(Luz_e_Cores)

