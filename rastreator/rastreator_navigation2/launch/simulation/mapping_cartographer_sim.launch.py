# Copyright (c) 2020
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Based on script by Gary Liu at Intel Corporation (realsense2)

# /* Authors: Iñaki Lorente */

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.actions.execute_process import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

#=====================================
#             VARIABLES
#=====================================
pkg_name = 'rastreator_description'
# rviz
dir_rviz = 'rviz'
rviz_config = "mapping.rviz"


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    realsense_prefix = get_package_share_directory('rastreator_navigation2')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(realsense_prefix, 'param'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='mapper_cartographer_lidar.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    #rviz
    rviz_file = os.path.join(get_package_share_directory(pkg_name),
                        dir_rviz,
                        rviz_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),

        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        ExecuteProcess(
            name='START-SIM',
            cmd=['ros2', 'launch', 'rastreator_bringup', 'start_sim.launch.py'],
            output = 'screen',
            shell='True'
        ),
        Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            node_name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
            )
    ])