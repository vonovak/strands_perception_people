cmake_minimum_required(VERSION 2.8.3)
project(odometry_to_motion_matrix)

find_package(PkgConfig REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  nav_msgs
  roscpp
  tf
  tf_conversions
  visual_odometry
)
find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    eigen_conversions
    nav_msgs
    roscpp
    tf
    tf_conversions
    visual_odometry
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom2visual src/odom2visual/main.cpp)

add_dependencies(odom2visual ${catkin_EXPORTED_TARGETS})

target_link_libraries(odom2visual
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS odom2visual
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
