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
#        LAUNCH CODE: tf2
#=====================================
def generate_launch_description():

    return LaunchDescription([
        Node(
            ## Configure the TF of the robot to the origin of the map coordinates
           package='tf2_ros',
           node_executable='static_transform_publisher',
           output='screen',
            #[x y z qx qy qz qw frame_id child_frame_id]
           arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
           )
    ])
