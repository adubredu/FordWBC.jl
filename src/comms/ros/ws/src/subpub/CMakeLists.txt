cmake_minimum_required(VERSION 3.0.2)
project(subpub) 

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(lcm REQUIRED )
 

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
)

 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES subpub
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)
 
include_directories(include  ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/subpub.cpp
# )


add_executable(${PROJECT_NAME}_node src/subpub.cpp src/main.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} lcm
)
