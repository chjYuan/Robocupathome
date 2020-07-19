cmake_minimum_required(VERSION 2.8.3)
project(from2Dto3D)
add_compile_options(-std=c++11)

# Catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  rospy
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  tf
  image_geometry
  image_transport
  geometry_msgs
  perception_msgs
#>>>>TODO: Add all the dependencies you need. Don't forget to update also the package manifest (package.xml).

)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io sample_consensus)

add_service_files(
  DIRECTORY srv
  FILES
  PointBase.srv
  ReachGoal.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


include_directories(include
  ${catkin_INCLUDE_DIRS}  
  include
  ${roscpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS}) 


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES from2Dto3D
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridges image_transport tf image_geometry sensor_msgs perception_msgs geometry_msgs
)

#>>>>TODO: Create a library for the class declared in the files.
#add_library(from2Dto3D_class src/from2Dto3D.cpp)
#target_link_libraries(from2Dto3D_class ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
#>>>>TODO: Link the executable with the library.

add_library(from2Dto3D_class src/from2Dto3D.cpp)
target_link_libraries(from2Dto3D_class ${roscpp_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(from2Dto3D_class ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(from2dto3d src/from2Dto3D_node.cpp)
target_link_libraries(from2dto3d
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  from2Dto3D_class
)
add_dependencies(from2dto3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


