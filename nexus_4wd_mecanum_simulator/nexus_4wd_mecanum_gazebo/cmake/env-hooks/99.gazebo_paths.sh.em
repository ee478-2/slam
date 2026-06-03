@[if DEVELSPACE]@
# In devel-space the package source dir is what holds media/ — point Gazebo there.
export GAZEBO_RESOURCE_PATH=@(CMAKE_CURRENT_SOURCE_DIR):${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=@(CMAKE_CURRENT_SOURCE_DIR)/models:${GAZEBO_MODEL_PATH}
@[else]@
# Install-space: media gets installed to share/<pkg>.
export GAZEBO_RESOURCE_PATH=@(CMAKE_INSTALL_PREFIX)/share/@(PROJECT_NAME):${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=@(CMAKE_INSTALL_PREFIX)/share/@(PROJECT_NAME)/models:${GAZEBO_MODEL_PATH}
@[end if]@
