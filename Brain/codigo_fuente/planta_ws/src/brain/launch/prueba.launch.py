import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('brain')
    return LaunchDescription([
        Node(package='brain', executable='brain_node', name='brain', output='screen',
             parameters=[os.path.join(share, 'config', 'brain_params.yaml')]),
        Node(package='brain', executable='robots_falsos', name='robots_falsos', output='screen',
             parameters=[os.path.join(share, 'config', 'robots_falsos_params.yaml')]),
    ])
