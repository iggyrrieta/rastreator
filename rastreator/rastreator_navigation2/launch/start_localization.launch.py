# Copyright 2020
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
#
# Authors: Iñaki Lorente

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

#=====================================
#             VARIABLES
#=====================================
# Package (where to find configs)
pkg_name = 'rastreator_navigation2'
# Folder inside package to find yaml
param_folder = 'param'
param_file = 'localization.yaml'
# Folder inside package to find map
map_folder = 'map'
map_file = 'casa_cartographer.yaml'

#=====================================
#        LAUNCH CODE: Navigation2
#=====================================
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Parameters
    params_dir = LaunchConfiguration(
        'params_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
                 param_folder,
                 param_file))

    # Map
    map_config_dir = LaunchConfiguration(
        'map_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
                 map_folder,
                 map_file))

    # Nav2 dir
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_config_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=params_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_launch.py']),
            launch_arguments={
                'map': map_config_dir,
                'use_sim_time': use_sim_time,
                'params': params_dir}.items(),
        )
    ]) 

