@[if DEVELSPACE]@
export GAZEBO_RESOURCE_PATH=@(CMAKE_CURRENT_SOURCE_DIR)/re540_final_map:@(CMAKE_CURRENT_SOURCE_DIR)/nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=@(CMAKE_CURRENT_SOURCE_DIR)/re540_final_map/models:${GAZEBO_MODEL_PATH}
@[else]@
export GAZEBO_RESOURCE_PATH=@(CMAKE_INSTALL_PREFIX)/share/@(PROJECT_NAME)/re540_final_map:@(CMAKE_INSTALL_PREFIX)/share/@(PROJECT_NAME)/nexus_4wd_mecanum_simulator/nexus_4wd_mecanum_gazebo:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=@(CMAKE_INSTALL_PREFIX)/share/@(PROJECT_NAME)/re540_final_map/models:${GAZEBO_MODEL_PATH}
@[end if]@
