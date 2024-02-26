import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('rs_to_velodyne')
    

    config = os.path.join(
        share_dir,
        'config',
        'param.yaml'
        )

    return LaunchDescription([
        Node(
            package='rs_to_velodyne',
            executable='rs_to_velodyne',
            name='rs_converter',
            parameters=[config],
            output='screen'
        )
    ])
