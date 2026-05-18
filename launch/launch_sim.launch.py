import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'mysimrobot'

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory(package_name), 'worlds', 'teknofest_arena.world'
        ),
        description='Full path to world file'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), 'config', 'twist_mux.yaml'
    )
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'gazebo_params.yaml'
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={
            'world': world,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mysimrobot'],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
    ])
