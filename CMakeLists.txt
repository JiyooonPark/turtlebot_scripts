cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_scripts)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS actionlib move_base_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

##add_executable(simple_navigation src/simple_navigation.cpp)
##target_link_libraries(turtlebot_scripts ${catkin_LIBRARIES})
