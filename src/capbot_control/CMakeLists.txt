cmake_minimum_required(VERSION 3.0.2)
project(capbot_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    roscpp
    hardware_interface
    diff_drive_controller
    joint_state_controller
    trajectory_msgs
)


catkin_package(
#  INCLUDE_DIRS 
#    include
  CATKIN_DEPENDS
    controller_manager
    hardware_interface
    roscpp
    trajectory_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

add_executable(capbot_hardware_interface_node src/capbot_hardware_interface_node.cpp)
target_link_libraries(capbot_hardware_interface_node ${catkin_LIBRARIES})
