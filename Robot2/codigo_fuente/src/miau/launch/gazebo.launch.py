import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = 'miau'
    pkg_share = FindPackageShare(pkg_name)
    
    # === OBTENER LA RUTA DE INSTALACIÓN DEL PAQUETE ===
    pkg_share_path = get_package_share_directory(pkg_name)
    
    # === CONFIGURAR VARIABLE DE ENTORNO PARA GAZEBO ===
    # Añadimos la ruta del paquete a GZ_SIM_RESOURCE_PATH para que resuelva 'model://miau/...'
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    parent_of_share = os.path.dirname(pkg_share_path)  # .../install/miau/share

    if gz_resource_path:
        new_gz_path = parent_of_share + ':' + gz_resource_path
    else:
        new_gz_path = parent_of_share
    set_gz_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_path)

    # === RUTA DEL URDF ===
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

    # --- Levanta Gazebo Harmonic con un mundo vacío ---
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
            '-name', 'miau',
            '-z', '0.05',
        ],
        output='screen',
    )

    # --- Puente de reloj de simulación Gazebo -> ROS2 ---
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # --- Controladores (arrancan un poco después de spawnear) ---
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
        # Primero configuramos la variable de entorno
        set_gz_env,
        # Luego el resto de acciones
        gazebo,
        robot_state_publisher_node,
        clock_bridge,
        spawn_entity,
        delayed_controllers,
    ])