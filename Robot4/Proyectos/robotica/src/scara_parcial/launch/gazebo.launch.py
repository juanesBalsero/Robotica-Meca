import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

# Obtenemos la ruta raíz donde ROS instala los paquetes (install/share)
pkg_share = get_package_share_directory('scara_parcial')
install_dir = os.path.abspath(os.path.join(pkg_share, '..'))

# Le decimos a Gazebo dónde buscar modelos y meshes
set_gz_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=install_dir
)

def generate_launch_description():
    pkg_name = 'scara_parcial'
    
    # Ruta al archivo URDF
    urdf_file = os.path.join(
        get_package_share_directory(pkg_name),
        'urdf',
        'Ensamblaje1.urdf'
    )

    # Leer contenido del URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Publicar la descripción del robot en ROS 2
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 2. Iniciar Gazebo Harmonic (un mundo vacío por defecto)
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Spawn del robot dentro del mundo de Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'scara_robot',
            '-z', '0.1'  # Elevación inicial respecto al suelo
        ],
        output='screen'
    )

    # 4. Puente de reloj para sincronizar ROS 2 y Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
    set_gz_resource_path,  # <--- Agrega esto al inicio
    node_robot_state_publisher,
    gazebo_sim,
    spawn_robot,
    bridge
])