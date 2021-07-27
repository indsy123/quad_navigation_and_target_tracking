cmake_minimum_required(VERSION 3.0.2)
project(quad_navigation_and_target_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

option(CXX "enable C++ compilation" ON)
if(CXX)
    enable_language(CXX)
endif()


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  trajectory_msgs
  std_msgs
  message_filters
  tf2_ros
  tf
  mavros_msgs
  mav_msgs
  pcl_ros
  message_generation
)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
#set(PCL_DIR "/home/ind/pcl-pcl-1.11.0/build/PCLConfig.cmake")
find_package(PCL 1.11 REQUIRED)
find_package(GSL REQUIRED) 

#catkin_python_setup()

add_message_files(
   FILES
   DetectionData.msg
   box_coordinates.msg
   publish_bounding_boxes.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quad_navigation_and_target_tracking
  CATKIN_DEPENDS message_runtime
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  trajectory_msgs
  std_msgs
  tf
  tf2_ros
  mavros_msgs
  mav_msgs
  pcl_ros
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/quad_navigation_and_target_tracking
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})


add_executable(planner_noRHP_node
  src/planner_noRHP_node.cpp
  src/trajectoryGenerationOptimalControl.cpp
  #src/ensembleGeneration.cpp
  src/planner_noRHP.cpp
  src/controller_simulation.cpp
  src/controller_px4.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(planner_noRHP_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   GSL::gsl 
   GSL::gslcblas

)

add_executable(rhp_planner_node
  src/rhp_planner_node.cpp
  src/trajectoryGenerationOptimalControl.cpp
  src/ensembleGeneration.cpp
  src/rhp_planner.cpp
  src/controller_simulation.cpp
  src/controller_px4.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(rhp_planner_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   GSL::gsl 
   GSL::gslcblas

)

add_executable(rhp_movingtarget_node
  src/rhp_movingtarget_node.cpp
  src/trajectoryGenerationOptimalControlMovingTarget.cpp
  src/ensembleGeneration.cpp
  src/rhp_movingtarget.cpp
  src/controller_simulation.cpp
  src/controller_px4.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(rhp_movingtarget_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   GSL::gsl 
   GSL::gslcblas

)

add_executable(vio2cog_transform_node
  src/vio2cog_transform_node.cpp
  src/vio2cog_transform.cpp
)


target_link_libraries(vio2cog_transform_node
   ${catkin_LIBRARIES}
)
