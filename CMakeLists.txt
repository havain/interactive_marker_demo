cmake_minimum_required(VERSION 3.0.2)
project(interactive_marker_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  tf
  visualization_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES interactive_marker_demo
 CATKIN_DEPENDS interactive_markers roscpp tf visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/marker_demo.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
