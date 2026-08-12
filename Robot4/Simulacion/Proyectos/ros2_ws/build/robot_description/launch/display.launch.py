import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'robot_description'
    
    # Ruta al archivo URDF
    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'Ensamblaje1.urdf'
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

    # GUI con deslizadores para articulaciones
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])