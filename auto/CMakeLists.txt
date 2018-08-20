cmake_minimum_required(VERSION 2.8.3)
project(auto)

find_package(catkin REQUIRED COMPONENTS
  rospy
  pcl_conversions
  tf_conversions
  pcl_ros
  roscpp
  sensor_msgs
  darknet_ros_msgs
  custom_msgs
)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES projector
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/auto.cpp
# )

add_library(projector src/Projector.cpp)
add_dependencies(projector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(projector ${catkin_LIBRARIES})


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(object_positioner src/object_projector.cpp)
#set_target_properties(object_positioner PROPERTIES OUTPUT_NAME object_positioner PREFIX "")
add_dependencies(object_positioner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_positioner 
    projector
    ${catkin_LIBRARIES}
)
