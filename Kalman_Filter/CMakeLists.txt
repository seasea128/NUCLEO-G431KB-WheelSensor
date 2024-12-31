cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS on)

project(
  Kalman_Filter
  VERSION 1.0
  LANGUAGES C)

include(CTest)

if(BUILD_TESTING)
  add_subdirectory(tests)
  add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} --verbose)
endif()

add_library(Kalman_Filter STATIC)

target_sources(Kalman_Filter PRIVATE kalman_state.c)

target_link_libraries(Kalman_Filter m)

target_include_directories(Kalman_Filter PUBLIC ./include/)
