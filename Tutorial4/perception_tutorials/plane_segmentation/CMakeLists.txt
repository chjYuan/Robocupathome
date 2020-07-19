cmake_minimum_required(VERSION 2.8.3)
project(plane_segmentation)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  pcl_conversions
  pcl_ros
  roscpp
  dynamic_reconfigure
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  image_geometry
  sensor_msgs
  message_generation
)
#>>>>TODO: Add all the dependencies you need. Don't forget to update also the package manifest (package.xml).

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp dynamic_reconfigure sensor_msgs)
## Specify additional locations of header files
## Your package locations should be listed before other locations

#>>>>TODO: Add the other include locations.
include_directories(
  include
  ${catkin_INCLUDE_DIRS}  
  ${PCL_INCLUDE_DIRS}
)


#>>>>TODO: Create a library for the class declared in the files.
#>>>>TODO: Link the executable with the library.
add_library(PlaneSegmentationClass
            src/PlaneSegmentationClass.cpp)
target_link_libraries(PlaneSegmentationClass ${catkin_LIBRARIES})
#add_dependencies(PlaneSegmentationClass ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



## Declare a cpp executable
add_executable(PlaneSegmentation src/PlaneSegmentation.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(PlaneSegmentation ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} PlaneSegmentationClass)
#add_dependencies(PlaneSegmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} PlaneSegmentationClass)


