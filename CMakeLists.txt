cmake_minimum_required(VERSION 3.6)
project(Mini_IMU)

#set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp  JY901.h)
add_executable(Mini_IMU ${SOURCE_FILES})