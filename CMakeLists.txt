cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once, and is not re-generated if converter is
# called multiple times.
#
# User is free to modify the file as much as necessary
#

option(MAIN_LTO "Enable LTO on main application" OFF)
option(MAIN_GC_SECTIONS "Enable GC sections on main application (Remove unused code)" ON)
set(SensorID "0x1" CACHE STRING "Sensor ID in hex")

include(FetchContent)

# Fetch drivers if needed
FetchContent_Declare(
  lsm6ds3tr-c
  GIT_REPOSITORY https://github.com/STMicroelectronics/LSM6DS3TR-C-PID.git
  GIT_TAG v2.0.1)

FetchContent_MakeAvailable(lsm6ds3tr-c)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME NUCLEO-G431KB-WheelSensor)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

if(MAIN_LTO) 
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# Add linker flag for printing floats
add_link_options(-u _printf_float)

if(MAIN_GC_SECTIONS)
  add_link_options(-Wl,--gc-sections)
endif()

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add state library
add_subdirectory(State)

# Link directories setup
target_link_directories(
  ${CMAKE_PROJECT_NAME} PRIVATE # Add user defined library search paths
)

# Add sources to executable
target_sources(
  ${CMAKE_PROJECT_NAME}
  PRIVATE # Add user sources here
          ./Core/Src/platforms/vl53l4cd_platform.c
          ./Core/Src/platforms/lsm6ds3tr_platform.c
          ./Core/Src/tof.c
          ./Core/Src/imu.c
          ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.c
          ./Drivers/VL53L4CD_ULD_Driver/VL53L4CD_calibration.c
          ${lsm6ds3tr-c_SOURCE_DIR}/lsm6ds3tr-c_reg.c)

# Add include paths
target_include_directories(
  ${CMAKE_PROJECT_NAME}
  PRIVATE # Add user defined include paths
          ./Core/Inc/platforms/ ./Drivers/VL53L4CD_ULD_Driver/Inc
          ${lsm6ds3tr-c_SOURCE_DIR})

# Add project symbols (macros)
target_compile_definitions(
  ${CMAKE_PROJECT_NAME} PRIVATE # Add user defined symbols
)

# Add linked libraries
target_link_libraries(
  ${CMAKE_PROJECT_NAME} stm32cubemx # Add user defined libraries
  State
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SENSOR_ID=${SensorID})

add_custom_command(
  TARGET ${CMAKE_PROJECT_NAME}
  POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
          ${CMAKE_PROJECT_NAME}.bin)
