import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():
    gui = LaunchConfiguration('gui')
    server = LaunchConfiguration('server')
    world = LaunchConfiguration('world')
    model_database_uri = LaunchConfiguration('model_database_uri')
    qt_qpa_platform = LaunchConfiguration('qt_qpa_platform')
    qt_x11_no_mitshm = LaunchConfiguration('qt_x11_no_mitshm')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='four_wheeled_robot' #<--- CHANGE ME
    default_world_path = os.path.join(
        get_package_share_directory(package_name), 'worlds', 'training_map.world'
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={
                    'gui': gui,
                    'server': server,
                    'world': world,
                }.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-x', spawn_x,
                                   '-y', spawn_y,
                                   '-z', spawn_z,
                                   '-Y', spawn_yaw,
                                   '-entity', 'my_bot'],
                        output='screen')



    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Set to "false" to run headless.',
        ),
        DeclareLaunchArgument(
            'server',
            default_value='true',
            description='Set to "false" not to run gzserver.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world_path,
            description='Path to Gazebo world file.',
        ),
        DeclareLaunchArgument(
            'model_database_uri',
            default_value='',
            description='Gazebo model database URI (empty disables online model DB lookup).',
        ),
        DeclareLaunchArgument(
            'qt_qpa_platform',
            default_value='xcb',
            description='Qt platform for Gazebo GUI (xcb recommended on GNOME Wayland).',
        ),
        DeclareLaunchArgument(
            'qt_x11_no_mitshm',
            default_value='1',
            description='Set QT_X11_NO_MITSHM to reduce X11 shared-memory GUI issues.',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='Initial x pose of robot.',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.0',
            description='Initial y pose of robot.',
        ),
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.20',
            description='Initial z pose of robot.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Initial yaw pose of robot (radians).',
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value=model_database_uri,
        ),
        SetEnvironmentVariable(
            name='QT_QPA_PLATFORM',
            value=qt_qpa_platform,
        ),
        SetEnvironmentVariable(
            name='QT_X11_NO_MITSHM',
            value=qt_x11_no_mitshm,
        ),
        rsp,
        gazebo,
        spawn_entity,
    ])
