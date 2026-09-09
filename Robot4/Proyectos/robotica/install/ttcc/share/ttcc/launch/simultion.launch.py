import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ttcc'
    
    # Ruta al archivo URDF
    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'ensamble1.urdf'
    )

    # Leer contenido del URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Nodo Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # RViz2 (Sin los sliders de joint_state_publisher_gui para evitar conflictos)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz
    ])