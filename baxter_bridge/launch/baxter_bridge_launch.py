import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    rviz = os.path.join(
        get_package_share_directory('baxter_description'),
        'launch',
        'rviz_launch.py')

    description_launch = os.path.join(
        get_package_share_directory('baxter_description'),
        'launch',
        'baxter_state_publisher_launch.py')

    decl_spawn_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Whether to spawn RViz together with the bridge")

    decl_use_baxter_description = DeclareLaunchArgument(
        "use_baxter_description",
        default_value=str(os.environ['ROS_DISTRO'] != 'galactic'),
        description="Whether to use description embedded in Baxter, or local one")

    use_baxter_description = LaunchConfiguration("use_baxter_description")

    baxter_embedded = Node(package='baxter_bridge',executable='bridge',output='screen',
                           condition=IfCondition(use_baxter_description))

    baxter_local = GroupAction([Node(package='baxter_bridge',executable='bridge',output='screen',parameters=[{'use_baxter_description': False}]),
                                IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch))],
                                condition=UnlessCondition(use_baxter_description))

    return LaunchDescription([
        decl_spawn_rviz,
        decl_use_baxter_description,
        baxter_embedded,
        baxter_local,
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz))],
                    condition=IfCondition(LaunchConfiguration("rviz")))])
