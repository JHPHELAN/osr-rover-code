import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    osr_bringup_dir = get_package_share_directory('osr_bringup')
    params_file = os.path.join(osr_bringup_dir, 'config', 'nav2_params.yaml')

    ld = LaunchDescription()
    
    # transform tf2_ros
    ld.add_action(Node(
        package='tf2_ros',
        name='footprint_static_tf2',
        executable='static_transform_publisher',
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "1", "base_link", "base_footprint"],
        output='screen'
    ))

    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
    #         launch_arguments={'map': os.path.join(osr_bringup_dir, "maps/1816fell.yaml"),
    #                           'use_sim_time': 'false',
    #                           'autostart': 'true',
    #                           'params_file': params_file}.items()),
    # )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'localization_launch.py')),
            launch_arguments={'map': os.path.join(osr_bringup_dir, "maps/1816fell.yaml"),
                              'use_sim_time': 'false',
                              'autostart': 'true',
                              'params_file': params_file}.items()),
    )

    return ld
