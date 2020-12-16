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
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

#=====================================
#             VARIABLES
#=====================================
# Package (where to find URDF configs)
pkg_name = 'rastreator_description'
# Folder inside package to find model
dir_model = 'urdf'
model_name = 'rastreator.urdf'


#=====================================
#             LAUNCH CODE
#=====================================
def generate_launch_description():

    # Sim
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # URDF File
    urdf_file = os.path.join(get_package_share_directory(pkg_name),
                        dir_model,
                        model_name)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # RASTREATOR model
        Node(
            package="robot_state_publisher",
            node_executable="robot_state_publisher",
            node_name="robot_state_publisher",
            output='screen',
            parameters=[{' use_sim_time': use_sim_time}],
            arguments=[urdf_file]
            )
    ])    

