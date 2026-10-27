import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'antropomorfico_sim'
    pkg_share = get_package_share_directory(pkg_name)
    parent_dir = os.path.dirname(pkg_share)

    # Exportar ruta de mallas a Gazebo Harmonic
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{pkg_share}:{parent_dir}"
    )

    # Ruta y lectura del URDF
    urdf_path = os.path.join(pkg_share, 'urdf', 'antropomorfico.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_desc = robot_desc.replace('package://antropomorfico_sim', 'file://' + pkg_share)
    robot_desc = robot_desc.replace('model://antropomorfico_sim', 'file://' + pkg_share)
    robot_desc = robot_desc.replace('$(find antropomorfico_sim)', pkg_share)

    # Nodo Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # Lanzar Gazebo Harmonic
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf '}.items()
    )

    # Insertar el robot en Gazebo
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'antropomorfico_3dof',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Puente ROS-Gazebo Bridge
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # Spawner para emisor de estados
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawner para controlador de posiciones del brazo
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    # Cargar controladores solo cuando el robot esté spawnado
    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                joint_state_broadcaster_spawner,
                arm_controller_spawner
            ]
        )
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo_sim,
        robot_state_publisher_node,
        spawn_robot_node,
        gz_bridge_node,
        delay_controllers_after_spawn
    ])
