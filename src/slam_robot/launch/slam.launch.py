import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_robot')
    slam_share = get_package_share_directory('slam_toolbox')

    # Disable FastDDS shared-memory transport (unreliable in containers);
    # UDPv4 only via FASTRTPS_DEFAULT_PROFILES_FILE.
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = os.path.join(
        pkg_share, 'config', 'fastrtps_udp.xml')

    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([slam_toolbox, rviz])
