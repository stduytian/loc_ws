import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 声明 PCD 文件路径参数
    declare_pcd_file_cmd = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/dongkuo/lt_ws/loc_ws/src/FAST_LIO/PCD/dog1.pcd',  # 默认为空，需要用户指定
        description='Path to the PCD file to publish'
    )
    
    declare_downsample_cmd = DeclareLaunchArgument(
        'downsample_factor',
        default_value='20',
        description='Downsample factor for the PCD file'
    )
    

    pcd_file = LaunchConfiguration('pcd_file')
    downsample_factor = LaunchConfiguration('downsample_factor')

    # PCD 发布者节点
    pcd_publisher_node = Node(
        package='fast_lio',
        executable='pcd_publisher',
        name='pcd_publisher',
        output='screen',
        parameters=[{
            'pcd_file': pcd_file,
            "downsample_factor": downsample_factor
        }],
    )

    # 创建启动描述
    ld = LaunchDescription()
    ld.add_action(declare_pcd_file_cmd)
    ld.add_action(declare_downsample_cmd)
    ld.add_action(pcd_publisher_node)

    return ld
