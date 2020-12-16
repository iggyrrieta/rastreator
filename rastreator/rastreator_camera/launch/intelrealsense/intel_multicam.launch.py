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

"""Launch realsense2_camera node without rviz2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        # Realsense T265
        launch_ros.actions.Node(
            package='realsense2_camera', 
            node_namespace='t265',
            node_executable='realsense2_camera_node',
            parameters = [{'device_type': 't265',
                           'base_frame_id':'t265_link',
                           'odom_frame_id':'t265_link',
                           'pose_frame_id':'t265_pose',
                           'pose_optical_frame_id':'t265_pose_optical',
                           'gyro_frame_id':'t265_gyro',
                           'imu_optical_frame_id':'t265_imu',
                           'accel_frame_id':'t265_accel',
                           'accel_optical_frame_id':'t265_accel_optical',
                           'enable_fisheye1': False,
                           'enable_fisheye2': False,
                           'topic_odom_in': 'odom',
                           'calib_odom_file': '',
                           'unite_imu_method': 'linear_interpolation',
                           }],
            output='screen',
            emulate_tty=True,
            ),
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
            ),
        # Transform depth camera to laserscan
        Node(
            package='depthimage_to_laserscan',
            node_executable='depthimage_to_laserscan_node',
            node_name='scan',
            output='screen',
            parameters=[{'output_frame':'d435_link'}],
            remappings=[('depth','/d435/camera/depth/image_rect_raw'),
                        ('depth_camera_info', '/d435/camera/depth/camera_info')],
            )
    ])