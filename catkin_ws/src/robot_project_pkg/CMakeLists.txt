cmake_minimum_required(VERSION 3.0.2)
project(robot_project_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joint_state_publisher_gui
  roscpp
  rviz
  tf2
  sensor_msgs
  stage_ros
  tf
)

find_package(OpenCV REQUIRED)

## Define catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_project_pkg 
  CATKIN_DEPENDS geometry_msgs joint_state_publisher_gui roscpp rviz tf2   sensor_msgs stage_ros
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  ${OpenCV_INCLUDE_DIRS}
)

## Declare C++ executables
add_executable(ObstacleVisualizer code/ObstacleVisualizer.cpp)
add_executable(Mapviewer code/Map_viewer.cpp)


## Link libraries to executables
target_link_libraries(ObstacleVisualizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(Mapviewer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


## Install targets
install(TARGETS ObstacleVisualizer Mapviewer 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Testing with gtest (if needed)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_project_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Python testing (if needed)
# catkin_add_nosetests(test)

