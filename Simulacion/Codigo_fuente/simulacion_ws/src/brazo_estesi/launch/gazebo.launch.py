from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('brazo_estesi')
    xacro_file = PathJoinSubstitution([pkg, 'urdf', 'brazo_estesi.urdf.xacro'])
    world = PathJoinSubstitution([pkg, 'worlds', 'empty.sdf'])

    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

    use_sim_time = LaunchConfiguration('use_sim_time')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world]}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'brazo_estesi',
            '-topic', 'robot_description',
            '-z', '0.0',
            '-allow_renaming', 'true',
        ],
    )

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'brazo_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
    )

    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb])
    )
    start_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb, on_exit=[arm_controller])
    )

    return LaunchDescription([
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz,
        rsp,
        spawn,
        start_jsb_after_spawn,
        start_arm_after_jsb,
    ])
