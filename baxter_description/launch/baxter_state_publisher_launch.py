import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():    
    
    urdf = os.path.join(
        get_package_share_directory('baxter_description'),
        'urdf',
        'baxter.urdf.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': xacro.process(urdf)}],
            remappings={'/joint_states': '/robot/joint_states'}.items()),
    ])
