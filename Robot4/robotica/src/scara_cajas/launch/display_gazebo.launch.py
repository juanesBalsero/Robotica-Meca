import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
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

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    load_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_position_controller'],
        output='screen'
    )

    gazebo_gui_node = Node(
        package='scara_cajas',
        executable='gazebo_mod',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    trajectory_node = Node(
        package='scara_cajas',
        executable='trajectory_exec',
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_scara,
        load_joint_state_broadcaster,
        load_position_controller,
        gazebo_gui_node,
        trajectory_node
    ])