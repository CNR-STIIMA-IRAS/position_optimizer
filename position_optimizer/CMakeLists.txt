cmake_minimum_required(VERSION 2.8.3)
project(position_optimizer)

set(CMAKE_BUILD_TYPE Debug)
find_package(catkin REQUIRED COMPONENTS
    rospy
    manipulability_pkg_msgs
    tf
    geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS
  DEPENDS)
    
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
