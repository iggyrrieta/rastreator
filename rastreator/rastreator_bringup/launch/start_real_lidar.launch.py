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

#=====================================
#             LAUNCH CODE
#=====================================
def generate_launch_description():
    # Sim
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Launch folder
    launch_file_dir = os.path.join(get_package_share_directory(pkg_name), dir_model)

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        ExecuteProcess(
            name='START-RASTREATOR',
            cmd=['ros2', 'launch', 'rastreator_wheelmotor', 'start.launch.py'],
            output = 'screen',
            shell='True'
        ),
        ExecuteProcess(
            name='START-JOYSTICK',
            cmd=['ros2', 'launch', 'rastreator_joystick', 'start.launch.py'],
            output = 'screen',
            shell='True'
        ),
        ExecuteProcess(
            name='START-LIDAR',
            cmd=['ros2', 'launch', 'rastreator_bringup', 'lidar.launch.py'],
            output = 'screen',
            shell='True'
        )
    ])