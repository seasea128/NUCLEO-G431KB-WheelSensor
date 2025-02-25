cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

project(
  State 
  VERSION 1.0
  LANGUAGES C)

include(CTest)

option(STATE_BUILD_TEST "Build tests for state library" OFF)

if(STATE_BUILD_TEST)
  add_subdirectory(tests)
  add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} --verbose)
endif()

add_library(State STATIC)

target_sources(State PRIVATE state.c)

# Link with math library
target_link_libraries(State m)

target_include_directories(State PUBLIC ./include/)
