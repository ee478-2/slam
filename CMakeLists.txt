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
    graph_planner/include
    local_costmap_generator/include
    nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/include
    nexus_4wd_mecanum_simulator/realsense_gazebo_plugin/include
  LIBRARIES
    heightmap_nodelet
    nexus_ros_force_based_move
    box_movement_plugin
    realsense_gazebo_plugin
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
  graph_planner/include
  local_costmap_generator/include
  nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/include
  nexus_4wd_mecanum_simulator/realsense_gazebo_plugin/include
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

add_executable(graph_planner_node graph_planner/src/graph_planner.cpp)
target_link_libraries(graph_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

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

add_library(nexus_ros_force_based_move
  nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/src/nexus_ros_force_based_move.cpp
)
target_link_libraries(nexus_ros_force_based_move
  ${Boost_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(box_movement_plugin
  nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/src/box_movement_plugin.cpp
)
target_link_libraries(box_movement_plugin ${GAZEBO_LIBRARIES})

add_library(realsense_gazebo_plugin
  nexus_4wd_mecanum_simulator/realsense_gazebo_plugin/src/RealSensePlugin.cpp
  nexus_4wd_mecanum_simulator/realsense_gazebo_plugin/src/gazebo_ros_realsense.cpp
)
target_link_libraries(realsense_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS
  localization_manager/scripts/localization_manager_node.py
  nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/src/control_plugin.py
  nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/src/gt_odom_publisher.py
  nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo/src/true_pose_marker.py
  re540_final_map/scripts/fix_world_paths.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  apriltag_localization_node
  graph_planner_node
  heightmap_node
  heightmap_costmap_node
  heightmap_nodelet
  nexus_ros_force_based_move
  box_movement_plugin
  realsense_gazebo_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  AprilTagLocalization/config
  AprilTagLocalization/launch
  AprilTagLocalization/rviz
  graph_planner/csv
  graph_planner/launch
  local_costmap_generator/launch
  localization_manager/config
  localization_manager/launch
  my_rtabmap/launch
  nexus_4wd_mecanum_simulator
  re540_final_map
  rviz
  scripts
  submission
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  local_costmap_generator/nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/local_costmap_generator
)
