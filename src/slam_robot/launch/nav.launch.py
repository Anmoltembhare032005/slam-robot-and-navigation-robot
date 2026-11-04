import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_robot')
    nav_share = get_package_share_directory('nav2_bringup')

    # Disable FastDDS shared-memory transport (unreliable in containers);
    # UDPv4 only via FASTRTPS_DEFAULT_PROFILES_FILE.
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = os.path.join(
        pkg_share, 'config', 'fastrtps_udp.xml')

    nav_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg_share, 'maps', 'slam_map.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav.rviz')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav_params,
            'use_sim_time': 'true',
            'autostart': 'true',
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

    return LaunchDescription([nav2, rviz])
