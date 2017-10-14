cmake_minimum_required(VERSION 2.8.3)
project(webot)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy sensor_msgs std_msgs tf message_generation PCL)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ric_gps.msg
   ric_command.msg
   ric_pan_tilt.msg
   ric_status.msg
   ric_raw.msg
   ric_elevator_command.msg
   ric_rc.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
 set_odom.srv
 ric_calib.srv
 set_elevator.srv
 home_elevator.srv
 relays.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()
#   DEPENDENCIES
#   geometry_msgs#   sensor_msgs#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS message_runtime roscpp geometry_msgs sensor_msgs std_msgs std_srvs tf)
   #INCLUDE_DIRS include
 #  CATKIN_DEPENDS message_runtime
 #LIBRARIES ric
#  CATKIN_DEPENDS geometry_msgs gps_common roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
add_executable(webot_node src/webot_node.cpp)

##add_executable(moveit_group_api src/moveit_group_api.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(webot_node ${PROJECT_NAME}_generate_messages_cpp)
##add_dependencies(moveit_group_api ${PROJECT_NAME}_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(webot_node
   ${catkin_LIBRARIES}
)

##target_link_libraries(moveit_group_api
##   ${catkin_LIBRARIES}
##)
#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS webot_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install (DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

