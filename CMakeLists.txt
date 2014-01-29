cmake_minimum_required(VERSION 2.8.3)
project(cob_mmcontroller)


find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs geometry_msgs trajectory_msgs visualization_msgs nav_msgs cob_srvs actionlib actionlib_msgs eigen_conversions)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#######################################
## Declare ROS messages and services ##
#######################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS rospy roscpp std_msgs geometry_msgs trajectory_msgs visualization_msgs nav_msgs cob_srvs actionlib actionlib_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(cob_config_controller src/cob_config_controller.cpp src/augmented_solver.cpp)
add_dependencies(cob_config_controller ${PROJECT_NAME}_gencpp)
#add_executable(cob_spnav_filter src/cob_spnav_filter.cpp)


###########
## Test ##
###########
# rostest


#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS cob_mmcontroller
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

