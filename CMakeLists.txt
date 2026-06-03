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
  pluginlib
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
    local_costmap_generator/include
  LIBRARIES
    heightmap_nodelet
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
  local_costmap_generator/include
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

add_executable(heightmap_node
  local_costmap_generator/src/heightmap_node.cpp
  local_costmap_generator/src/heightmap.cpp
)
target_link_libraries(heightmap_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(heightmap_costmap_node
  local_costmap_generator/src/heightmap_to_costmap.cpp
)
target_link_libraries(heightmap_costmap_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(heightmap_nodelet
  local_costmap_generator/src/heightmap_nodelet.cpp
  local_costmap_generator/src/heightmap.cpp
)
target_link_libraries(heightmap_nodelet
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(PROGRAMS
  localization_manager/scripts/localization_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  apriltag_localization_node
  heightmap_node
  heightmap_costmap_node
  heightmap_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  AprilTagLocalization/config
  AprilTagLocalization/launch
  AprilTagLocalization/rviz
  config
  local_costmap_generator/launch
  localization_manager/config
  localization_manager/launch
  my_rtabmap/launch
  rviz
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  local_costmap_generator/nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/local_costmap_generator
)
