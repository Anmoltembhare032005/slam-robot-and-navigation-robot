import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_robot')

    # Disable FastDDS shared-memory transport (unreliable in containers);
    # UDPv4 only via FASTRTPS_DEFAULT_PROFILES_FILE.
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = os.path.join(
        pkg_share, 'config', 'fastrtps_udp.xml')

    world_file = os.path.join(pkg_share, 'worlds', 'slam_room.sdf')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    robot_description = xacro.process_file(urdf_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-r -v 3 {world_file}',
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-name', 'slam_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.12',
        ],
    )

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ]

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=bridge_args,
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        gz_bridge,
    ])
