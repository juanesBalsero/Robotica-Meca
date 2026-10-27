import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_scara = get_package_share_directory('scara_cajas')
    urdf_file = os.path.join(pkg_scara, 'urdf', 'Ensamblaje1.urdf')
    controllers_yaml = os.path.join(pkg_scara, 'config', 'controllers.yaml')

    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    # Reemplaza el placeholder por la ruta absoluta real del yaml de controladores
    robot_description_config = robot_description_config.replace(
        'CONTROLLERS_YAML_PLACEHOLDER', controllers_yaml
    )

    # Permitir que Gazebo encuentre las mallas STL de 'package://scara_cajas/'
    pkg_parent_dir = os.path.dirname(pkg_scara)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_parent_dir]
    )

    # ---- Argumentos: por defecto corre las trayectorias de los Excel, sin sliders ----
    arg_trayectoria = DeclareLaunchArgument(
        'trayectoria', default_value='true',
        description='Ejecutar las trayectorias de los Excel (trajectory_gazebo)')
    arg_sliders = DeclareLaunchArgument(
        'sliders', default_value='false',
        description='Abrir la ventana de sliders (gazebo_mod). No usar junto a trayectoria.')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # /clock de Gazebo hacia ROS (lo necesitan los nodos con use_sim_time:=True)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    spawn_scara = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description_config,
            '-name', 'SCARA_Robot',
            '-z', '0.0'
        ],
        output='screen'
    )

    # El controller_manager solo existe cuando el robot ya esta cargado en Gazebo.
    # Se espera hasta 60 s en vez de los 10 s por defecto del spawner.
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '60'],
        output='screen'
    )

    load_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_position_controller',
                   '--controller-manager-timeout', '60'],
        output='screen'
    )

    gazebo_gui_node = Node(
        package='scara_cajas',
        executable='gazebo_mod',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('sliders'))
    )

    trajectory_node = Node(
        package='scara_cajas',
        executable='trajectory_gazebo',
        output='screen',
        condition=IfCondition(LaunchConfiguration('trayectoria'))
    )

    # Orden: robot -> joint_state_broadcaster -> scara_position_controller -> nodos
    return LaunchDescription([
        arg_trayectoria,
        arg_sliders,
        set_gz_resource_path,
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_scara,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_scara,
            on_exit=[load_joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_position_controller])),
        RegisterEventHandler(OnProcessExit(
            target_action=load_position_controller,
            on_exit=[trajectory_node, gazebo_gui_node])),
    ])