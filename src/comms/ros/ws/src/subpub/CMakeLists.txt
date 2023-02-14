cmake_minimum_required(VERSION 3.0.2)
project(subpub) 

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(JlCxx_DIR "/home/ford/alphonsus/software/libcxxwrap-julia/libcxxwrap-julia-build")
set(Julia_EXECUTABLE "/home/ford/alphonsus/software/julia-1.6.5-linux-x86_64/julia-1.6.5/bin/julia")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)
find_package(lcm REQUIRED )
find_package(JlCxx)
get_target_property(JlCxx_location JlCxx::cxxwrap_julia LOCATION)
get_filename_component(JlCxx_location ${JlCxx_location} DIRECTORY)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${JlCxx_location}")

add_message_files(
    FILES 
    SceneObject.msg 
    SceneObjectArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs#   nav_msgs#   sensor_msgs#   std_msgs
)

 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES subpub
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)
 

include_directories(include  ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(Subpub SHARED src/subpub.cpp)
target_link_libraries(Subpub JlCxx::cxxwrap_julia ${DEPENDENCIES} ${catkin_LIBRARIES})

# add_executable(${PROJECT_NAME}_node src/subpub.cpp src/main.cpp)


# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES} lcm
# )
