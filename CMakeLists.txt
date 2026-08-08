cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME solar_imu)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/App/Inc
    ${CMAKE_SOURCE_DIR}/Core/BSP/ML307C/Inc
    ${CMAKE_SOURCE_DIR}/Core/BSP/LSM6DS/Inc
    ${CMAKE_SOURCE_DIR}/Core/BSP/UART/Inc
    ${CMAKE_SOURCE_DIR}/Core/Protocol/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# 加入用户自定义的源文件路径
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Core/Inc
    # ${CMAKE_SOURCE_DIR}/BSP/Inc
)

# 加入用户自定义的源文件（排除已被stm32cubemx处理的文件）
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/Core/App/Src/app_controller.c
    ${CMAKE_SOURCE_DIR}/Core/BSP/ML307C/Src/at_ml307c.c
    ${CMAKE_SOURCE_DIR}/Core/BSP/LSM6DS/Src/lsm6ds.c
    ${CMAKE_SOURCE_DIR}/Core/BSP/LSM6DS/Src/lsm6ds3tr-c_reg.c
    ${CMAKE_SOURCE_DIR}/Core/BSP/UART/Src/uart_driver.c
    ${CMAKE_SOURCE_DIR}/Core/App/Src/event_store.c
    ${CMAKE_SOURCE_DIR}/Core/Protocol/Src/service_protocol.c
    ${CMAKE_SOURCE_DIR}/Core/App/Src/sys_config.c
    ${CMAKE_SOURCE_DIR}/Core/App/Src/power_manager.c
    ${CMAKE_SOURCE_DIR}/Core/App/Src/rtc_utils.c
    ${CMAKE_SOURCE_DIR}/Core/App/Src/service_handler.c
    ${CMAKE_SOURCE_DIR}/Core/App/Src/event_report.c
    ${CMAKE_SOURCE_DIR}/Core/App/Src/low_power.c
)

# 浮点数 printf 已移除 (MQTT/GPS函数已注释，无浮点格式化需求)
