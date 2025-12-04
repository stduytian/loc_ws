import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    use_pointcloud_to_laserscan = LaunchConfiguration('use_pointcloud_to_laserscan')
     # 点云转激光扫描节点
    pointcloud_to_laserscan_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/cloud_registered_body '),  # 直接使用原始点云
            ('scan', '/scan_body'),
        ],
        parameters=[{
            'use_sim_time': False,
            'scan_height': 1,
            'min_height': -0.1,  
            'max_height': 0.1,   
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087, # 0.5度
            'scan_time': 0.0,
            'range_min': 0.1,
            'range_max': 50.0,   
            'use_inf': False,  
            'inf_epsilon': 1.0,
            # 'target_frame': 'scan_frame',
            'transform_tolerance': 2.0,
            'transform_timeout': 2.0,
            'downsample_factor': 1,  
            'max_points_per_scan': 10000,  
            'use_scan_barycenter':  True,  # 扫描重心计算
            'use_high_fidelity_projection': True,  # 高保真投影
        }],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    ld = LaunchDescription()
    ld.add_action(pointcloud_to_laserscan_cmd)

    return ld
