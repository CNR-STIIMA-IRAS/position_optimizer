cmake_minimum_required(VERSION 2.8.3)
project(manipulability_pkg)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  object_loader_msgs
  manipulability_pkg_msgs
  roscpp
  descartes_moveit
  rosdyn_core
  moveit_planning_helper
  descartes_planner
  eigen_conversions
  )

find_package(Boost REQUIRED system filesystem date_time thread )
find_package(Eigen3)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    manipulability_pkg_msgs
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  LIBRARIES
)

include_directories(SYSTEM 
                    ${Boost_INCLUDE_DIR}
                    ${EIGEN3_INCLUDE_DIR} 
                    ${KUKA_FRI_INCLUDE}   
                    ${catkin_INCLUDE_DIRS}
                    include 
)

## Declare a cpp library
add_library(
trajectory src/trajectory.cpp
)
add_library(
pick_place_manipulability src/pick_place_manipulability.cpp
)

target_link_libraries(trajectory ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(pick_place_manipulability trajectory ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})

link_directories(${catkin_LIBRARY_DIRS})

add_executable(pick_place_manipulability_srv src/pick_place_manipulability_srv.cpp)
target_link_libraries(pick_place_manipulability_srv trajectory pick_place_manipulability ${catkin_LIBRARIES})
add_dependencies(pick_place_manipulability_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
