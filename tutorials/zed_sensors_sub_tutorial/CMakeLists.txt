cmake_minimum_required(VERSION 2.8.3)
project(zed_sensors_sub_tutorial)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

## Build 
add_executable(zed_sensors_sub src/zed_sensors_sub_tutorial.cpp)
target_link_libraries(zed_sensors_sub ${catkin_LIBRARIES})
