cmake_minimum_required(VERSION 2.8.3)
project(interactive_world)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS interactive_markers actionlib rospy roscpp message_generation)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GraspCheck.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

include_directories(
  /opt/ros/groovy/stacks/pr2_controllers/pr2_controllers_msgs/msg_gen/cpp/include
  /opt/ros/groovy/stacks/pr2_common/pr2_msgs/msg_gen/cpp/include
)

## Declare a cpp executable
add_executable(grasp_verification src/grasp_verification.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(grasp_verification ${catkin_LIBRARIES})

add_dependencies(grasp_verification ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

install(PROGRAMS scripts/interactive_world_server scripts/interactive_world_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS grasp_verification
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
