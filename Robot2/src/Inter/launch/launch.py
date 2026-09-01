#!/usr/bin/env python3
"""
launch.py — paquete "Inter"
============================
Lanza en RViz2 los dos robots del paquete al mismo tiempo, cada uno
con su propio joint_state_publisher_gui (sliders):

    - SCARA          -> origen (0, 0, 0)
    - RS003NFF60      -> origen (100, 700, 0) mm = (0.1, 0.7, 0) m

Los URDF se leen desde <share>/Inter/urdf/, es decir de la carpeta
"urdf" que ya tienes dentro del paquete (instalada vía CMakeLists).

Ambos URDF reutilizan los mismos nombres de link/joint
(base_link, link1, link2, link3, world...), así que cada robot corre
en su propio namespace de ROS2 ("scara" / "rs003") y con
"frame_prefix" para que robot_state_publisher no choque los tf de
un robot con los del otro. Cada robot conserva su cadena interna
"<ns>/world -> ...", y se ancla a un frame común "map" mediante
static_transform_publisher, en la posición pedida.

Requisitos: robot_state_publisher, joint_state_publisher_gui, rviz2,
tf2_ros (paquetes estándar de ros-<distro>-desktop).

Uso:
    ros2 launch inter launch.py
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'Inter'

# Nombres de archivo dentro de <share>/Inter/urdf/
SCARA_URDF_FILE = 'scara_robot.urdf'
RS003_URDF_FILE = 'rs003nff60.urdf'

# Posiciones pedidas (interpretadas en mm -> convertidas a metros)
SCARA_ORIGIN_XYZ = ('0', '0', '0')          # (0, 0, 0)
RS003_ORIGIN_XYZ = ('0.1', '0.7', '0')      # (100, 700, 0) mm


def _read_urdf(path: str) -> str:
    with open(path, 'r') as f:
        return f.read()


def generate_launch_description():

    pkg_share = get_package_share_directory(PACKAGE_NAME)
    scara_urdf_path = os.path.join(pkg_share, 'urdf', SCARA_URDF_FILE)
    rs003_urdf_path = os.path.join(pkg_share, 'urdf', RS003_URDF_FILE)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'dual_robot.rviz')

    scara_description = _read_urdf(scara_urdf_path)
    rs003_description = _read_urdf(rs003_urdf_path)

    # ---------------- SCARA -> origen (0,0,0) ----------------
    scara_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='scara',
        output='screen',
        parameters=[{
            'robot_description': scara_description,
            'frame_prefix': 'scara/',
        }],
    )

    scara_joint_slider = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace='scara',
        output='screen',
        parameters=[{'robot_description': scara_description}],
    )

    map_to_scara = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_scara_world',
        arguments=[*SCARA_ORIGIN_XYZ, '0', '0', '0', 'map', 'scara/world'],
    )

    # ---------------- RS003NFF60 -> origen (100,700,0) mm ----------------
    rs003_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='rs003',
        output='screen',
        parameters=[{
            'robot_description': rs003_description,
            'frame_prefix': 'rs003/',
        }],
    )

    rs003_joint_slider = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace='rs003',
        output='screen',
        parameters=[{'robot_description': rs003_description}],
    )

    map_to_rs003 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_rs003_world',
        arguments=[*RS003_ORIGIN_XYZ, '0', '0', '0', 'map', 'rs003/world'],
    )

    # ---------------- RViz2 ----------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        scara_state_publisher,
        scara_joint_slider,
        map_to_scara,
        rs003_state_publisher,
        rs003_joint_slider,
        map_to_rs003,
        rviz_node,
    ])
