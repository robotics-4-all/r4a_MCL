cmake_minimum_required(VERSION 2.8.3)
project(art_localization_particle_filters_ogm)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  message_generation
  stdr_msgs
  tf
  )
find_package(PkgConfig)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  particleInitSrv.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  )


catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES particle_filter
  CATKIN_DEPENDS 
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  stdr_msgs
  #  DEPENDS system_lib
  )
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(robot_perception_lib
  src/robot_perception.cpp
  )
add_dependencies(robot_perception_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  art_localization_particle_filters_ogm_gencpp
  )

add_library(particle_lib
  src/particle.cpp
  )
add_dependencies(particle_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  art_localization_particle_filters_ogm_gencpp
  )

add_library(particle_filter_lib
  src/particle_filter.cpp
  )
add_dependencies(particle_filter_lib 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  art_localization_particle_filters_ogm_gencpp
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
add_executable(localization src/main.cpp)
target_link_libraries(localization
  particle_filter_lib
  robot_perception_lib
  particle_lib
  ${catkin_LIBRARIES}
  )

add_dependencies(localization 
  ${catkin_EXPORTED_TARGETS}
  art_localization_particle_filters_ogm_gencpp
  )

