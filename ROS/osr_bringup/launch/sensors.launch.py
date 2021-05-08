import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'sick_lms_5xx.yaml'
        )
    # first run `ifconfig eth0 192.168.4.1 netmask 255.255.255.0 up`
    node=Node(
        package='sick_scan2',
        name = 'sick_scan2', # 'sick_scan2_lms_5xx', # For compatibility with ros versions previous to foxy, node name changed to sick_scan2 for all supported scanner. The type of scanner is configured by scanner_name in the yaml config file.
        node_executable='sick_generic_caller',       # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable".
        # executable='sick_generic_caller',          # Please use executable='sick_generic_caller', if ROS2 can't launch sick_generic_caller.
        output='screen',
        parameters = [config]
    )
    ld.add_action(node)
    # lidar tf2_ros
    ld.add_action(Node(
        package='tf2_ros',
        name='laser_tf2',
        executable='static_transform_publisher',
        arguments=["0.01", "0.03", "0.25", "0", "0", "0", "1", "base_link", "cloud"],
        output='screen'
    ))
    ld.add_action(Node(
        package='v4l2_camera',
        name='front_camera_node',
        executable='v4l2_camera_node',
        parameters=[
            {'video_device': '/dev/video0'},
            {'pixel_format': 'YUYV'},
            {'camera_frame_id': 'front_webcam'},
            {'output_encoding': 'rgb8'},
            {'image_size': [1280, 720]}
        ]
    ))
    # front camera tf2_ros
    ld.add_action(Node(
        package='tf2_ros',
        name='camera_tf2',
        executable='static_transform_publisher',
        arguments=["0.12", "0.03", "0.22", "0", "0", "0", "1", "base_link", "front_webcam"],
        output='screen'
    ))

    return ld
