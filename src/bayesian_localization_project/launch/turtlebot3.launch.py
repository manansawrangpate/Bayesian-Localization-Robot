import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_share = get_package_share_directory('bayesian_localization_project')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    world = os.path.join(project_share, 'worlds', 'project_world.sdf')
    model_sdf = os.path.join(tb3_gazebo_share, 'models', 'turtlebot3_waffle_pi', 'model.sdf')

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        # Append to GZ_SIM_RESOURCE_PATH so Gazebo can resolve model:// mesh URIs
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(tb3_gazebo_share, 'models')
        ),

        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),

        # Gazebo server (headless, runs physics)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world],
                'on_exit_shutdown': 'true',
            }.items()
        ),

        # Gazebo client (GUI window)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g -v2'}.items()
        ),

        # Robot state publisher (TF transforms + robot_description topic)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_share, 'launch', 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Spawn TurtleBot3 after world is ready
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=[
                        '-name', 'waffle_pi',
                        '-file', model_sdf,
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', '0.01',
                    ]
                )
            ]
        ),

        # ROS-Gazebo bridge for non-image topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            output='screen'
        ),

        # Separate image bridge required for camera images in Gazebo Harmonic
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen',
        ),
    ])