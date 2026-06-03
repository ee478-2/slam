@[if DEVELSPACE]@
# Devel-space: world/materials/models live under the source tree.
export GAZEBO_RESOURCE_PATH=@(CMAKE_CURRENT_SOURCE_DIR):${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=@(CMAKE_CURRENT_SOURCE_DIR)/models:${GAZEBO_MODEL_PATH}
@[else]@
# Install-space: assets land in share/<pkg>.
export GAZEBO_RESOURCE_PATH=@(CMAKE_INSTALL_PREFIX)/share/@(PROJECT_NAME):${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=@(CMAKE_INSTALL_PREFIX)/share/@(PROJECT_NAME)/models:${GAZEBO_MODEL_PATH}
@[end if]@
