cmake_minimum_required(VERSION 2.8.3)
project(from2dto3d)

 add_compile_options(-std=c++11)

# Catkin
find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  tf
  image_geometry
  darknet_ros_msgs
  perception_msgs

#>>>>TODO: Add all the dependencies you need. Don't forget to update also the package manifest (package.xml).

)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)



catkin_package(
#INCLUDE_DIRS include
#LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS message_runtime
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


#>>>>TODO: Create a library for the class declared in the files.

#>>>>TODO: Link the executable with the library.
add_library(from2Dto3DClass
            src/from2Dto3DClass.cpp)
target_link_libraries(from2Dto3DClass ${catkin_LIBRARIES})

add_executable(from2dto3d src/from2Dto3D.cpp)
add_dependencies(from2dto3d ${from2dto3d_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(from2dto3d
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  from2Dto3DClass
)

install(TARGETS from2dto3d
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

