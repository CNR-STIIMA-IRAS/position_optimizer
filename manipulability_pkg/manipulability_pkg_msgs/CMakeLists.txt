cmake_minimum_required(VERSION 2.8.3)
project(manipulability_pkg_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_service_files( DIRECTORY  srv
FILES pickPlaceManipulability.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES   geometry_msgs std_msgs )

catkin_package(    CATKIN_DEPENDS geometry_msgs message_generation std_msgs )
