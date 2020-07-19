cmake_minimum_required(VERSION 2.8.3)
project(object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  roscpp
  sensor_msgs
  std_msgs
  darknet_ros_msgs
  perception_msgs
  tf
)
find_package(OpenCV REQUIRED)
find_package(darknet_ros_msgs REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)






###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_recognition
)

include_directories(
  ${catkin_INCLUDE_DIRS}  
  include
  ${roscpp_INCLUDE_DIRS}
  
)


add_library(recognition_class src/ObjectRecognition.cpp)
target_link_libraries(recognition_class ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(recognition_class ${recognition_class_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# node 

add_executable(objectRecognition src/ObjectRecognition_node.cpp)
target_link_libraries(objectRecognition ${catkin_LIBRARIES} ${roscpp_LIBRARIES}  recognition_class)
add_dependencies(objectRecognition ${objectRecognition_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
