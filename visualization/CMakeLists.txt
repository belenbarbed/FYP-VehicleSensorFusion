cmake_minimum_required(VERSION 2.8.3)
project(visualization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  face_detection
  image_geometry
  image_transport
  message_filters
  message_generation
  message_runtime
  pcl_conversions
  pcl_msgs
  pcl_ros
  rosconsole
  roscpp
  roslib
  roslint
  rospy
  sensor_msgs
  std_msgs
  velodyne_pointcloud
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  pcl_msgs
  sensor_msgs
  std_msgs
  face_detection
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visualization
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_geometry image_transport message_filters message_generation message_runtime pcl_conversions pcl_msgs pcl_ros rosconsole roscpp roslib roslint rospy sensor_msgs std_msgs velodyne_pointcloud
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

#############
##   C++   ##
#############

# join_pointclouds
add_executable(join_pointclouds src/join_pointclouds.cpp)
add_dependencies(join_pointclouds ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(join_pointclouds ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# video_visualization
add_executable(video_visualization src/video_visualization.cpp src/Image.cpp src/Velodyne.cpp)
add_dependencies(video_visualization ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(video_visualization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# Mark executables and/or libraries for installation
install(TARGETS join_pointclouds video_visualization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
