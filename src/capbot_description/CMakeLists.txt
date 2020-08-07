cmake_minimum_required(VERSION 3.0.2)
project(capbot_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  urdf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES capbot_description
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(parser src/parser.cpp)
target_link_libraries(parser ${catkin_LIBRARIES})

