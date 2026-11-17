"""Robot 1 completo para la planta: udp_bridge + kinematic_control + trajectory_publisher
+ robot1_node (contrato con el brain).

    ros2 launch robot_control robot1_planta.launch.py
    ros2 launch robot_control robot1_planta.launch.py params_file:=/ruta/otro.yaml

No reemplaza a robot.launch.py (que sigue sirviendo para probar el robot solo).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_control')
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'params.yaml'),
                              description='YAML con los parámetros de los nodos'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'robot.launch.py')),
            launch_arguments={'params_file': params}.items()),
        Node(package='robot_control', executable='robot1_node', name='robot1_node',
             parameters=[params], output='screen', emulate_tty=True),
    ])
