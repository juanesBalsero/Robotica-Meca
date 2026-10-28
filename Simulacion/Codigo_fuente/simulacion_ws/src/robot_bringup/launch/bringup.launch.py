import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def resolve_urdf_paths(urdf_string):
    """
    1. Reemplaza $(find paquete) por la ruta de instalación absoluta.
    2. Reemplaza package://paquete/ por file:///ruta_absoluta/share/paquete/.
    """
    find_pattern = r'\$\(find\s+([^\)]+)\)'
    def replace_find(match):
        pkg_name = match.group(1).strip()
        try:
            return get_package_share_directory(pkg_name)
        except Exception:
            return match.group(0)
    urdf_string = re.sub(find_pattern, replace_find, urdf_string)

    pkg_pattern = r'package://([^/]+)/'
    def replace_package(match):
        pkg_name = match.group(1).strip()
        try:
            pkg_share = get_package_share_directory(pkg_name)
            return f'file://{pkg_share}/'
        except Exception:
            return match.group(0)
    
    return re.sub(pkg_pattern, replace_package, urdf_string)

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_robot_bringup, 'worlds', 'empty_world.sdf')

    # Registra dinámicamente el directorio share de cada paquete activo en GZ_SIM_RESOURCE_PATH
    ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')
    share_paths = [os.path.join(p, 'share') for p in ament_prefix_path.split(':') if p]
    gz_resource_path_val = ':'.join(share_paths)
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        gz_resource_path_val += ':' + os.environ['GZ_SIM_RESOURCE_PATH']

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path_val
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Ruta absoluta al archivo de mundo SDF'
    )

    # Lanza Gazebo automáticamente despausado (-r)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f"-r {world_file}"}.items()
    )

    # Puente de reloj explícito entre Gazebo Harmonic y ROS 2
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/default/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/world/default/clock', '/clock')],
        output='screen'
    )

    robots_config = [
        {
            'name': 'antropomorfico',
            'pkg': 'antropomorfico_sim',
            'file': 'urdf/antropomorfico.urdf',
            'x': '0.38', 'y': '0.385', 'z': '0.0'
        },
        {
            'name': 'miau',
            'pkg': 'miau',
            'file': 'urdf/Completo.urdf',
            'x': '0.35', 'y': '0.86', 'z': '0.0'
        },
        {
            'name': 'brazo_estesi',
            'pkg': 'brazo_estesi',
            'file': 'urdf/brazo_estesi.urdf.xacro',
            'x': '0.57', 'y': '0.67', 'z': '0.0'
        },
        {
            'name': 'scara_boceto',
            'pkg': 'scara_boceto',
            'file': 'urdf/scara_boceto.urdf.xacro',
            'x': '0.9', 'y': '0.76', 'z': '0.0'
        },
        {
            'name': 'scara_description',
            'pkg': 'scara_description',
            'file': 'urdf/scara.urdf.xacro',
            'x': '0.85', 'y': '0.46', 'z': '0.0'
        }
    ]

    nodes = [gz_resource_path, world_arg, gz_sim, clock_bridge]

    for bot in robots_config:
        try:
            bot_pkg_dir = get_package_share_directory(bot['pkg'])
            model_path = os.path.join(bot_pkg_dir, bot['file'])

            if model_path.endswith('.xacro'):
                robot_desc = xacro.process_file(model_path).toxml()
            else:
                with open(model_path, 'r') as f:
                    robot_desc = f.read()

            robot_desc = resolve_urdf_paths(robot_desc)

            rsp_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=bot['name'],
                output='screen',
                parameters=[{
                    'robot_description': robot_desc,
                    'use_sim_time': True,
                    'frame_prefix': f"{bot['name']}/"
                }]
            )

            spawn_node = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'default',
                    '-name', bot['name'],
                    '-topic', f"/{bot['name']}/robot_description",
                    '-x', bot['x'],
                    '-y', bot['y'],
                    '-z', bot['z']
                ],
                output='screen'
            )

            delayed_spawn = TimerAction(
                period=5.0,
                actions=[spawn_node]
            )

            nodes.append(rsp_node)
            nodes.append(delayed_spawn)

        except Exception as e:
            print(f"[WARN] No se pudo cargar la descripción del robot {bot['name']}: {e}")

    return LaunchDescription(nodes)