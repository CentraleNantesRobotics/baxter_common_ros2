import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():    
    
    config = os.path.join(
        get_package_share_directory('baxter_description'),
        'launch',
        'config.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d',config]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments="0 0 .9 0 0 0 ground base".split()),])
