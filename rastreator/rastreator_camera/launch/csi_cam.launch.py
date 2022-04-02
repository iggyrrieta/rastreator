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
from launch_ros.actions import Node

#=====================================
#             VARIABLES
#=====================================
# Package (where to find configs)
pkg_name = 'rastreator_camera'
# Folder inside package to find yaml
param_folder = 'param'
param_file = 'picam_conf.yaml'

#=====================================
#    LAUNCH CODE: raspberry PI-CAMERA
#=====================================
def generate_launch_description():

    # Parameters
    params_dir = LaunchConfiguration(
        'params_dir',
        default=os.path.join(
            get_package_share_directory(pkg_name),
                 param_folder,
                 param_file))

    return LaunchDescription([
        # Camera 
        Node(
            package='rastreator_camera',
            executable='csi_streamer',
            parameters=[params_dir],
            output='screen'),
        # Compress image 
        Node(
            package='image_transport',
            executable='republish',
            arguments=['raw', 'raw'],
            remappings=[('in','/image_raw'),
                        ('out','/image_raw/compressed')],
            output='screen')
    ])    

