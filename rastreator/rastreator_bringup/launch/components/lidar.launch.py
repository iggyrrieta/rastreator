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
from launch import LaunchDescription
from launch_ros.actions import Node

#=====================================
#        LAUNCH CODE: lidar
#=====================================
def generate_launch_description():

    return LaunchDescription([
        # RPLIDAR
         Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB1', #ttyUSBx
                'serial_baudrate': 256000,  # A3
                'frame_id': 'lidar_link',
                'inverted': False,
                'angle_compensate': False, # Esto deberia estar a true, pero a true peta
                'scan_mode': 'Sensitivity'
                #'angle_min': 0, #Por defecto 0º
                #'angle_max': 360 #Por defecto 360º
            }]),
    ])    

