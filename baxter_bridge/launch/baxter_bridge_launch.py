import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    rviz = os.path.join(
        get_package_share_directory('baxter_description'),
        'launch',
        'rviz_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Whether to spawn RViz together with the bridge"),
        Node(
            package='baxter_bridge',
            executable='bridge',
            output='screen'),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz))],
                    condition=IfCondition(LaunchConfiguration("rviz")))])
