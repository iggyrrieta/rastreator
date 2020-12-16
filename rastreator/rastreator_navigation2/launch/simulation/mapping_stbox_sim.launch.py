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
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess

from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

#=====================================
#             VARIABLES
#=====================================
# Package (where to find LAUNCH files)
pkg_name = 'rastreator_bringup'
# Folder inside package to find launch files
dir_model = 'launch/components'
# Package (where to find WORLD files)
pkg_name2 = 'rastreator_description'
# Folder inside package2 to find world files
dir_world = 'world'
world_file_name = 'small_house/rastreator_home'
# rviz
dir_rviz = 'rviz'
rviz_config = "mapping.rviz"

#=====================================
#             LAUNCH CODE
#=====================================
def generate_launch_description():
    # Sim
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Launch folder
    launch_file_dir = os.path.join(get_package_share_directory(pkg_name), dir_model)
    # Description folder (to get world)
    world = os.path.join(get_package_share_directory(pkg_name2), 
                     dir_world, 
                     world_file_name)
    #rviz
    rviz_file = os.path.join(get_package_share_directory(pkg_name2),
                        dir_rviz,
                        rviz_config)

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        ExecuteProcess(
            name='START-SIM',
            cmd=['ros2', 'launch', 'rastreator_bringup', 'start_sim.launch.py'],
            output = 'screen',
            shell='True'
        ),
        ExecuteProcess(
            name='START-NAV2',
            cmd=['ros2', 'launch', 'rastreator_navigation2', 'start_navigation.launch.py'],
            output = 'screen',
            shell='True'
        ),
        Node(
        parameters=[get_package_share_directory("rastreator_navigation2") + '/param/mapper_stbox.yaml',
          {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        node_executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            output='screen',
            arguments=['-d', rviz_file]
            )
    ])