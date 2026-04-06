import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('four_wheeled_robot')
    map_default = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    # Doi bien nay de chon thuat toan mac dinh:
    # 'astar' hoac 'dijkstra'
    thuat_toan_mac_dinh = 'astar'
    nav2_params_default = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    rviz_default = os.path.join(pkg_share, 'config', 'nav2_robot_view.rviz')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'localization_launch.py',
            )
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': 'False',
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_custom.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'planner_algorithm': LaunchConfiguration('planner_algorithm'),
            'use_respawn': 'False',
            'log_level': 'info',
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

    initial_pose_helper = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='four_wheeled_robot',
                executable='set_initial_pose_once',
                name='set_initial_pose_once',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': True,
                        'x': spawn_x,
                        'y': spawn_y,
                        'yaw': spawn_yaw,
                        'frame_id': 'map',
                        'repeat_count': 4,
                        'period_sec': 0.5,
                    }
                ],
                condition=IfCondition(LaunchConfiguration('auto_initial_pose')),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('server', default_value='true'),
            DeclareLaunchArgument(
                'world',
                default_value=os.path.join(pkg_share, 'worlds', 'training_map.world'),
            ),
            DeclareLaunchArgument('spawn_x', default_value='0.0'),
            DeclareLaunchArgument('spawn_y', default_value='0.0'),
            DeclareLaunchArgument('spawn_z', default_value='0.20'),
            DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
            DeclareLaunchArgument('map', default_value=map_default),
            DeclareLaunchArgument('nav2_params_file', default_value=nav2_params_default),
            DeclareLaunchArgument(
                'planner_algorithm',
                default_value=thuat_toan_mac_dinh,
                description='Custom global planner algorithm: astar or dijkstra.',
            ),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('auto_initial_pose', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('rviz_config', default_value=rviz_default),
            sim,
            localization,
            navigation,
            initial_pose_helper,
            rviz,
        ]
    )
