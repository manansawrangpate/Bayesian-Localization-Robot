import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    project_share = get_package_share_directory('bayesian_localization_project')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    tb3_src_root = os.path.expanduser('~/gazebo_projects/src/turtlebot3')
    tb3_sim_models = os.path.expanduser('~/gazebo_projects/src/turtlebot3_simulations/turtlebot3_gazebo/models')
    tb3_description_root = os.path.join(tb3_src_root, 'turtlebot3_description')
    tb3_model_sdf = os.path.join(tb3_sim_models, 'turtlebot3_waffle_pi', 'model.sdf')

    gz_resource_path = ':'.join([
        tb3_sim_models,
        tb3_src_root,
        tb3_description_root,
    ])

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),

        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(project_share, 'worlds', 'project_world.sdf')
        ),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.2'),
        DeclareLaunchArgument('yaw', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r ', world]
            }.items()
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=[
                        '-world', 'mail_robot_track',
                        '-name', 'turtlebot3',
                        '-file', tb3_model_sdf,
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', z_pose
                    ]
                )
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            output='screen'
        ),
    ])