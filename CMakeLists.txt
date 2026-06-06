cmake_minimum_required(VERSION 3.0.2)
project(slam)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  angles
  camera_info_manager
  cv_bridge
  gazebo_ros
  geometry_msgs
  image_transport
  nav_msgs
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS
    AprilTagLocalization/include
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    nodelet
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
)

include_directories(
  AprilTagLocalization/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_add_env_hooks(99.slam_gazebo_paths
  SHELLS sh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake/env-hooks
)

add_executable(apriltag_localization_node
  AprilTagLocalization/src/apriltag_localization_node.cpp
  AprilTagLocalization/src/apriltag_localization.cpp
)
target_link_libraries(apriltag_localization_node ${catkin_LIBRARIES} yaml-cpp)

catkin_install_python(PROGRAMS
  localization_manager/scripts/localization_manager_node.py
  scripts/mission_viz_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  apriltag_localization_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  AprilTagLocalization/config
  AprilTagLocalization/launch
  AprilTagLocalization/rviz
  config
  launch
  localization_manager/config
  localization_manager/launch
  my_rtabmap/launch
  rviz
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
