import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
RETARDO_NODO = 8.0


def generate_launch_description():
    pkg_share = get_package_share_directory('miau')

    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'trayectoria.yaml'),
        description='YAML con puntos, secuencia y velocidades')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py')))

    nodo = Node(
        package='miau',
        executable='nodo_jacobiano',
        name='trajectory_jacobian_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {'use_sim_time': True,
             'log_dir': os.path.expanduser('~/miau_logs')},   # aquí caen los CSV
        ],
    )

    return LaunchDescription([
        declare_params,
        gazebo,
        TimerAction(period=RETARDO_NODO, actions=[nodo]),
    ])
