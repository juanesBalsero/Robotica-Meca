from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = FindPackageShare('scara_description')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'scara.urdf.xacro'])
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'))
    )
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot_description], output='screen')
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic','robot_description','-entity','scara'], output='screen')
    return LaunchDescription([gazebo, rsp, spawn])
