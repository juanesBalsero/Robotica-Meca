import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'Miau'
    pkg_share = FindPackageShare(pkg_name)
    model_path = PathJoinSubstitution([pkg_share, 'urdf', 'Completo.urdf'])

    robot_description_content = ParameterValue(
        Command(['xacro ', model_path]), value_type=str
    )

    # --- robot_state_publisher: publica /robot_description y los TF ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    # --- Levanta Gazebo Harmonic con un mundo vacio ---
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # --- Spawnea el robot en Gazebo leyendo /robot_description ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'Miau',
            '-z', '0.05',
        ],
        output='screen',
    )

    # --- Puente de reloj de simulacion Gazebo -> ROS2 ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # --- Controladores (arrancan un poco despues de spawnear, para que el plugin ya este cargado) ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen',
    )

    delayed_controllers = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, position_controller_spawner],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        clock_bridge,
        spawn_entity,
        delayed_controllers,
    ])
