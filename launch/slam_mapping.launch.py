import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('four_wheeled_robot')
    slam_params_default = os.path.join(
        pkg_share, 'config', 'slam_toolbox_online_async.yaml'
    )
    rviz_default = os.path.join(pkg_share, 'config', 'nav2_robot_view.rviz')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_sim.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'server': LaunchConfiguration('server'),
            'world': LaunchConfiguration('world'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': LaunchConfiguration('slam_params_file'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('server', default_value='true'),
            # đổi world 
            DeclareLaunchArgument(
                'world',
                default_value=os.path.join(pkg_share, 'worlds', 'warehouse_RIO_1.world'),
            ),
            DeclareLaunchArgument('spawn_x', default_value='0.0'),
            DeclareLaunchArgument('spawn_y', default_value='3.5'),
            DeclareLaunchArgument('spawn_z', default_value='0.35'),
            DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
            DeclareLaunchArgument(
                'slam_params_file',
                default_value=slam_params_default,
            ),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('rviz_config', default_value=rviz_default),
            sim,
            slam,
            rviz,
        ]
    )
