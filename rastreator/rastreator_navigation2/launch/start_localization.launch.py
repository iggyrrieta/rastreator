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
from launch.actions.execute_process import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

#=====================================
#             VARIABLES
#=====================================
# Package (where to find configs)
pkg_name = 'rastreator_navigation2'
# Folder inside package to find yaml
param_folder = 'param'
param_file = 'localization_sim.yaml'
# Folder inside package to find map
map_folder = 'map'
map_file = 'map.yaml'



#=====================================
#        LAUNCH CODE: Navigation2
#=====================================
def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') # True if sim
    autostart = LaunchConfiguration('autostart')
    lifecycle_nodes = ['map_server', 'amcl']


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

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_config_dir}

    configured_params = RewrittenYaml(
        source_file=params_dir,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

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
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    ]) 

