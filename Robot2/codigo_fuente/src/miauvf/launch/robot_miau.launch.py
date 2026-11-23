import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('miau')

    usar_cinematico = LaunchConfiguration('usar_cinematico')
    serial_port = LaunchConfiguration('serial_port')
    vel_max_prueba = LaunchConfiguration('vel_max_prueba')
    use_rviz = LaunchConfiguration('use_rviz')

    decl_cin = DeclareLaunchArgument(
        'usar_cinematico',
        default_value='true',
        description='false = solo GUI + sliders manuales; true = incluye nodo_cinematico para botones PH/PA/PB/P1/P2',
    )
    decl_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Puerto serial de la ESP32 Antena',
    )
    decl_vmax = DeclareLaunchArgument(
        'vel_max_prueba',
        default_value='0.80',
        description='Límite máximo [rad/s] de los sliders en la pestaña Pruebas',
    )
    decl_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Abrir RViz2 sincronizado con el robot real',
    )

    urdf_path = os.path.join(pkg_share, 'urdf', 'Completo.urdf')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'config.rviz')
    cin_yaml = os.path.join(pkg_share, 'config', 'cinematico.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    bridge_node = Node(
        package='miau',
        executable='esp32_robot_bridge.py',
        name='esp32_robot_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'serial_port': serial_port,
            'baudrate': 115200,
            'use_sim_time': False,
        }],
    )

    gui_node = Node(
        package='miau',
        executable='gui_miau.py',
        name='gui_miau',
        output='screen',
        parameters=[{
            'fuente_realimentacion': 'robot',
            'vel_max_prueba': ParameterValue(vel_max_prueba, value_type=float),
            'use_sim_time': False,
        }],
    )

    cinematico_node = Node(
        package='miau',
        executable='nodo_cinematico',
        name='nodo_cinematico',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(usar_cinematico),
        parameters=[
            cin_yaml,
            {
                'fuente_realimentacion': 'robot',
                'use_sim_time': False,
            },
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        decl_cin,
        decl_port,
        decl_vmax,
        decl_rviz,
        rsp_node,
        bridge_node,
        gui_node,
        rviz_node,
        TimerAction(period=3.0, actions=[cinematico_node]),
    ])
