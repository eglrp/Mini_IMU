cmake_minimum_required(VERSION 3.6)
project(Mini_IMU)

set(CMAKE_CXX_STANDARD 11)
#find_package(Boost REQUIRED COMPONENTS system
#        )
#set(Boost_USE_STATIC_LIBS        ON) # only find static libs
#set(Boost_USE_MULTITHREADED      ON)
#set(Boost_USE_STATIC_RUNTIME    OFF)
#include_directories(
#        ${Boost_INCLUDE_DIRS}
#
#)

#link_directories(
#        ${Boost_LIBRARY_DIRS}
#)
include_directories(
        ./include
)

set(SOURCE_FILES main.cpp  JY901.h)
add_executable(Mini_IMU ${SOURCE_FILES})
#target_link_libraries(Mini_IMU ${Boost_LIBRARIES})


add_executable(IMUPlus IMUPlus.cpp include/CharQueue.cpp include/CharQueue.h)