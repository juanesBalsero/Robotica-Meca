import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'Miau'

    pkg_share = FindPackageShare(pkg_name)
    model_path = PathJoinSubstitution([pkg_share, 'urdf', 'Completo.urdf'])
    rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])

    # robot_state_publisher necesita el contenido del URDF ya resuelto.
    # Si mas adelante usas xacro, cambia esta linea por Command(['xacro ', model_path]).
    robot_description_content = ParameterValue(
        Command(['xacro ', model_path]), value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
