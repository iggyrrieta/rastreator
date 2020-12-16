import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch_ros.actions import Node

#=====================================
#             VARIABLES
#=====================================
# Package (where to find configs)
pkg_name = 'rastreator_simulation'
# Folder inside package to find yaml
param_folder = 'param'
param_file = 'config.yaml'
# Package (where to find WORLD files)
pkg_name2 = 'rastreator_description'
# Folder inside package2 to find world files
dir_world = 'world'
world_file_name = 'empty_world/rastreator'

#=====================================
#        LAUNCH CODE: Simulation
#=====================================
def generate_launch_description():

    # Parameters
    params_dir = LaunchConfiguration(
        'params_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
                 param_folder,
                 param_file))
    # Description folder (to get world)
    world = os.path.join(get_package_share_directory(pkg_name2), 
                     dir_world, 
                     world_file_name)

    return LaunchDescription([
        Node(
            package='rastreator_simulation',
            node_executable='ekf',
            parameters=[params_dir],
            output='screen'),

        Node(
            package='rqt_plot',
            node_executable='rqt_plot',
            arguments=['/ekf/estimated_state/x:y', '/ekf/true_state/x:y'],
            output='screen'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen'),

    ])    

