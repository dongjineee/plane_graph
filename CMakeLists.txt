cmake_minimum_required(VERSION 3.0.2)
project(qp_slam)

set(CMAKE_CXX_STANDARD 14)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_filters
  tf2_ros
  tf2_sensor_msgs
  visualization_msgs
  message_generation
  rospy
  vision_msgs

  cv_bridge
  image_geometry
  image_transport
  pcl_conversions
)

# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES graph_slam
# #  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs
# #  DEPENDS system_lib
# )

add_message_files(
   FILES
   plane.msg
   planes.msg
   key_m.msg
   pose.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   nav_msgs
   ) #custom message에 추가할 메시지형태 선언

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs nav_msgs
)

find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
)

link_directories(include
${GTSAM_LIBRARY_DIRS}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIR})

catkin_install_python(PROGRAMS
  quadric_script/transform.py
  quadric_script/detection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(tracker_cloud src/tracker_cloud.cpp )
target_link_libraries(tracker_cloud 
${catkin_LIBRARIES} 
${OpenCV_LIBRARIES})

add_executable(plane_frontend_node src/front_plane.cpp )
target_link_libraries(plane_frontend_node ${catkin_LIBRARIES})

add_executable(keyframe_node src/keyframe.cpp)
target_link_libraries(keyframe_node ${catkin_LIBRARIES})

add_executable(backend_node src/backend.cpp)
target_link_libraries(backend_node ${catkin_LIBRARIES} 
gtsam)
