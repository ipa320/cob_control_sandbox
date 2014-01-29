cmake_minimum_required(VERSION 2.8.3)
project(cob_mmcontroller)


find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs geometry_msgs nav_msgs trajectory_msgs cob_srvs actionlib actionlib_msgs kdl_parser kdl_conversions orocos_kdl tf tf_conversions eigen_conversions)


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
    DEPENDS rospy roscpp std_msgs geometry_msgs trajectory_msgs nav_msgs cob_srvs actionlib actionlib_msgs tf tf_conversions
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO
    LIBRARIES # TODO
) 

###########
## Build ##
###########

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(cob_config_controller src/cob_config_controller.cpp src/augmented_solver.cpp)
target_link_libraries(cob_config_controller ${catkin_LIBRARIES})

add_executable(cob_mmcontroller src/cob_mmcontroller.cpp)
target_link_libraries(cob_mmcontroller ${catkin_LIBRARIES})

add_executable(cob_spnav_filter src/cob_spnav_filter.cpp)
target_link_libraries(cob_spnav_filter ${catkin_LIBRARIES})

add_executable(kuka_test src/kuka_test.cpp src/augmented_solver.cpp)
target_link_libraries(kuka_test ${catkin_LIBRARIES})


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

