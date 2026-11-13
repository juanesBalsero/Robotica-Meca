"""Lanza el sistema completo: udp_bridge + kinematic_control + trajectory_publisher.

    ros2 launch robot_control robot.launch.py
    ros2 launch robot_control robot.launch.py params_file:=/ruta/otro.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    por_defecto = os.path.join(
        get_package_share_directory('robot_control'), 'config', 'params.yaml')

    params = LaunchConfiguration('params_file')

    def nodo(ejecutable):
        return Node(package='robot_control', executable=ejecutable, name=ejecutable,
                    parameters=[params], output='screen', emulate_tty=True)

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=por_defecto,
                              description='YAML con los parámetros de los nodos'),
        nodo('udp_bridge'),
        nodo('kinematic_control'),
        nodo('trajectory_publisher'),
    ])
