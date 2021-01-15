import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    voc_file = LaunchConfiguration('voc_file')

    remappings = [
        ('/camera/image_raw', '/image_raw'),
        ('/camera/camera_info', '/camera_info'),
        ('/orb_slam2_mono_node/debug_image', '/cam_points'),
        ('/orb_slam2_mono_node/map', '/map_points')
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory("rastreator_camera"),
                'param', 'orb_csi_mono_localization.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),

        DeclareLaunchArgument(
            'voc_file',
            default_value=os.path.join(
                get_package_share_directory("orb_slam2_ros"),
                'orb_slam2', 'Vocabulary', 'ORBvoc.txt'),
            description='Full path to vocabulary file to use'),

        Node(
            parameters=[
              params_file,
              {"voc_file": voc_file,
               "use_sim_time": use_sim_time},
            ],
            package='orb_slam2_ros',
            node_executable='orb_slam2_ros_mono',
            node_name='orb_slam2_mono',
            output='screen',
            remappings=remappings
        )
    ])
