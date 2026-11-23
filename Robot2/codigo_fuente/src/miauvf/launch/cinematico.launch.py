import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('miau')

    fuente = LaunchConfiguration('fuente')
    es_gazebo = PythonExpression(["'", fuente, "' == 'gazebo'"])

    declarar_fuente = DeclareLaunchArgument('fuente', default_value='gazebo')
    declarar_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'cinematico.yaml'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        condition=IfCondition(es_gazebo))

    nodo = Node(
        package='miau',
        executable='nodo_cinematico',
        name='nodo_cinematico',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_sim_time': ParameterValue(es_gazebo, value_type=bool),
                'fuente_realimentacion': ParameterValue(fuente, value_type=str),
            },
        ],
    )

    return LaunchDescription([
        declarar_fuente,
        declarar_params,
        gazebo,
        TimerAction(period=8.0, actions=[nodo]),
    ])
