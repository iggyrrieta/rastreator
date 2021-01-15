# Local variables for ROS2 and Gazebo

`ROS2` framework is based on `CMake`. It means all changes need a compilation to work. As we can have multiple `ROS2` environments it is a good method to create local variables pointing to your working environment, this way all changes will be applied when running a new terminal.  

`ROS2` has different local variables that can be set also from the `~/.bashrc` file to get your environment up and running every time you run a new terminal.

Same way `Gazebo` has different local variables that can also be set from the `~/.bashrc` 



Open `~/.bashrc` with your favourite editor and insert the following:

```bash
#=========================================================================
#                         Iñaki Lorente @2020 
#=========================================================================

#=============
# Alias control
#=============
alias start_cam='ros2 launch rastreator_bringup start_real_csicam.launch.py'
alias orb_map='ros2 launch rastreator_camera orb_slam2_csi_map_launch.py'
alias orb_loc='ros2 launch rastreator_camera orb_slam2_csi_loc_launch.py'

#=============
# ROS | ROS2
#=============
export ROS_DISTRO=eloquent 
export ROS_dir=ros        #CHANGE 1: THIS USING YOUR ROS2 FOLDER NAME

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/$ROS_dir/$ROS_DISTRO/install/local_setup.bash

# Domain (security, any number you want to use)
export ROS_DOMAIN_ID=21

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

#=============
# GAZEBO
#=============
source /usr/share/gazebo/setup.sh

# Folders containing the models and worlds
export RASTREATOR_DESCRIPTION=~/$ROS_dir/$ROS_DISTRO/install/rastreator_description/share/rastreator_description

export GAZEBO_PLUGINS=~/$ROS_dir/$ROS_DISTRO/install/gazebo_plugins/lib
# Sourcing /usr/share/gazebo/setup.sh is not enough cause we need to append
# more locations to it
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-9:$RASTREATOR_DESCRIPTION/world:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:/opt/ros/$ROS_DISTRO/lib:$GAZEBO_PLUGINS:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=/usr/share/gazebo-9/models:$RASTREATOR_DESCRIPTION/model:${GAZEBO_MODEL_PATH}

```

> If you are using Ubuntu, the only change needed is the one marked as `CHANGE 1`