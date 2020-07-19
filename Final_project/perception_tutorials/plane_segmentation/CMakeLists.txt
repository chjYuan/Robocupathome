cmake_minimum_required(VERSION 2.8.3)
project(plane_segmentation)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf
  image_geometry
  sensor_msgs
  geometry_msgs

#>>>>TODO: Add all the dependencies you need. Don't forget to update also the package manifest (package.xml).

)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io sample_consensus)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS}) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plane_segmentation
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridges image_transport tf image_geometry sensor_msgs geometry_msgs
)
## Specify additional locations of header files
## Your package locations should be listed before other locations

#>>>>TODO: Add the other include locations.
include_directories(
  ${catkin_INCLUDE_DIRS}  
  include
  ${roscpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


#>>>>TODO: Create a library for the class declared in the files.

#>>>>TODO: Link the executable with the library.
add_library(plane_segmentation_class src/PlaneSegmentation.cpp)
target_link_libraries(plane_segmentation_class ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
#set_target_properties(plane_segmentation_class  PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# node 

add_executable(plane_segmentation src/PlaneSegmentation_node.cpp)
target_link_libraries(plane_segmentation ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${PCL_LIBRARIES} plane_segmentation_class)
add_dependencies(plane_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
