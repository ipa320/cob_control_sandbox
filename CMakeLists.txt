cmake_minimum_required(VERSION 2.8.3)
project(cob_mmcontroller)


find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs geometry_msgs nav_msgs trajectory_msgs cob_srvs actionlib actionlib_msgs kdl_parser tf tf_conversions eigen_conversions brics_actuator control_msgs)


find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs trajectory_msgs nav_msgs control_msgs cob_srvs actionlib actionlib_msgs tf tf_conversions
    DEPENDS # TODO
    INCLUDE_DIRS # TODO
    LIBRARIES # TODO
) 

###########
## Build ##
###########

include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})

add_executable(cob_config_controller src/cob_config_controller.cpp src/augmented_solver.cpp)
target_link_libraries(cob_config_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(cob_mmcontroller src/cob_mmcontroller.cpp)
target_link_libraries(cob_mmcontroller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(cob_spnav_filter src/cob_spnav_filter.cpp)
target_link_libraries(cob_spnav_filter ${catkin_LIBRARIES})

add_executable(kuka_test src/kuka_test.cpp src/augmented_solver.cpp)
target_link_libraries(kuka_test ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(manipulability_publisher src/manipulability_publisher.cpp)
target_link_libraries(manipulability_publisher ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(trajectory_manager src/trajectory_manager.cpp)
target_link_libraries(trajectory_manager ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

###########
## Test ##
###########
# rostest


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cob_config_controller cob_mmcontroller cob_spnav_filter kuka_test manipulability_publisher trajectory_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(PROGRAMS src/dual_arm_script.py src/dual_arm_test.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
