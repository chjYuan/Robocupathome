cmake_minimum_required(VERSION 2.8.3)
project(ikTiago)

## Compile as C++11, supported in ROS Kinetic and newer
if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trac_ik_lib
  tf
  message_generation
  actionlib
  control_msgs
)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(orocos_kdl REQUIRED)

add_service_files(
   FILES
   gripper.srv
   doorinfo.srv
 )

 generate_messages(
  DEPENDENCIES
  geometry_msgs  #   perception_msgs#   sensor_msgs#   std_msgs
 )


catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ikTiago 
  tf_tiago
  CATKIN_DEPENDS
    trac_ik_lib
  DEPENDS
    Boost
    orocos_kdl
    tf_tiago
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(ikTiago src/ikTiago.cpp)
target_link_libraries(ikTiago
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${tf_tiago_LIBRARIES}
)
add_dependencies(ikTiago  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
install(TARGETS ikTiago
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


add_executable(little_move src/littlemove.cpp)
target_link_libraries(little_move
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${orocos_kdl_LIBRARIES}
${tf_tiago_LIBRARIES}
)
add_dependencies(little_move  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable(openwide src/openwide.cpp)
target_link_libraries(openwide
${catkin_LIBRARIES}
${Boost_LIBRARIES}
${orocos_kdl_LIBRARIES}
${tf_tiago_LIBRARIES}
)
add_dependencies(openwide  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
