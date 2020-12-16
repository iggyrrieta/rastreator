# Copyright (c) 2018 Intel Corporation
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

"""Launch realsense2_camera node without rviz2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    return LaunchDescription([
        # Realsense D435
        launch_ros.actions.Node(
            package='realsense2_camera', 
            node_namespace='d435',
            node_executable='realsense2_camera_node',
            parameters = [{'device_type': 'd435',
                           'base_frame_id':'d435_link',
                           'color_frame_id':'d435_color',
                           'infra1_frame_id':'d435_infra1',
                           'depth_frame_id':'d435_depth',
                           'depth_optical_frame_id':'d435_depth_optical',
                           'color_optical_frame_id':'d435_color_optical',
                           'aligned_depth_to_color_frame_id':'d435_depth_2_color',
                           'aligned_depth_to_infra1_frame_id':'d435_depth_2_infra1',
                           'infra1_optical_frame_id':'d435_infra1_optical',
                           'infra2_frame_id':'d435_infra2',
                           'infra2_optical_frame_id':'d435_infra2_optical',
                           'align_depth': True,
                           'enable_pointcloud': True,
                           'color_width': 640,
                           'color_height': 480,
                           'depth_width': 640,
                           'depth_height': 480,
                           'clip_distance': -2.0,
                           }],
            output='screen',
            emulate_tty=True,
            )
    ])