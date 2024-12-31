cmake_minimum_required(VERSION 3.22)

project(Kalman_Filter_Test LANGUAGES C)

include(CTest)

include(FetchContent)
FetchContent_Declare(
  unity
  GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
  GIT_TAG v2.6.0)
FetchContent_MakeAvailable(unity)

add_executable(test1)

target_sources(test1 PRIVATE test1.c)

target_link_libraries(test1 Kalman_Filter unity)

add_test(test1_test test1)
