import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def generate_launch_description():
    laser_link_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_to_laser',
        arguments=['0', '0', '0', '0', '0.289', '0', 'laser_link', 'livox_frame'],
        output='screen'
    )

    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.11854', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    base_link_to_laser_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_link',
        arguments=['-0.55851', '0', '-0.38617', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen'
    )
    
    livox_to_scan = Node(
        package='point2d_loc',
        executable='livox_to_scan',
        name='livox_to_scan'
    )

    return LaunchDescription([
        laser_link_to_livox_frame,
        base_footprint_to_base_link,
        base_link_to_laser_link,
        livox_to_scan
    ])
