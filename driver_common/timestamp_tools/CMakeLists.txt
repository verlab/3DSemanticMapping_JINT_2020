# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(timestamp_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED roslib roscpp)
find_package(Boost REQUIRED)

include_directories(include)

catkin_package(
  DEPENDS roslib roscpp
  INCLUDE_DIRS include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")


if(CATKIN_ENABLE_TESTING)
	catkin_add_gtest(test_trigger_matcher test/test_trigger_matcher.cpp)
	target_link_libraries(test_trigger_matcher ${catkin_LIBRARIES})
endif()
