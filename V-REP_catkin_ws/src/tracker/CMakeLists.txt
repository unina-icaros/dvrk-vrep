cmake_minimum_required(VERSION 2.8.3)
project(tracker_cpp)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visp_bridge
  visp_ros
  message_generation
##  genmsg
)

find_package(VISP REQUIRED)

# Add package definitions
#add_definitions(${VISP_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################
catkin_package( 
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_geometry
    roscpp
    sensor_msgs
    std_msgs
    visp_bridge
    visp_ros
    message_runtime

  DEPENDS
    VISP
)

###################
## Build library ##
###################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

#################
## Build nodes ##
#################
## Declare a cpp executable
add_executable(tracker_blobs_only tracker_blobs_only.cpp)
add_executable(tracker_blobs_ibvs tracker_blobs_ibvs.cpp)
add_executable(tracker_blobs_ibvs_real tracker_blobs_ibvs_real.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tracker_blobs_only ${catkin_LIBRARIES})
target_link_libraries(tracker_blobs_ibvs ${catkin_LIBRARIES})
target_link_libraries(tracker_blobs_ibvs_real ${catkin_LIBRARIES})


