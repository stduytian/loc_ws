import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    declare_pcd_file_cmd = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/dongkuo/lt_ws/loc_ws/src/FAST_LIO/PCD/dog1.pcd',
        description='Path to the PCD file to convert'
    )

    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution', default_value='0.05',
        description='Map resolution in meters/cell'
    )

    declare_min_z_cmd = DeclareLaunchArgument(
        'min_z', default_value='0.0',
        description='Minimum Z (m) to include in projection'
    )

    declare_max_z_cmd = DeclareLaunchArgument(
        'max_z', default_value='0.15',
        description='Maximum Z (m) to include in projection'
    )

    declare_downsample_cmd = DeclareLaunchArgument(
        'downsample_factor', default_value='1',
        description='Downsample factor: take 1 every N points'
    )

    declare_padding_cmd = DeclareLaunchArgument(
        'padding', default_value='1.0',
        description='Padding (m) to expand map bounds'
    )

    declare_output_prefix_cmd = DeclareLaunchArgument(
        'output_prefix', default_value='map',
        description='Output file prefix (produces <prefix>.pgm and <prefix>.yaml)'
    )

    # LaunchConfiguration substitutions
    pcd_file = LaunchConfiguration('pcd_file')
    resolution = LaunchConfiguration('resolution')
    min_z = LaunchConfiguration('min_z')
    max_z = LaunchConfiguration('max_z')
    downsample_factor = LaunchConfiguration('downsample_factor')
    padding = LaunchConfiguration('padding')
    output_prefix = LaunchConfiguration('output_prefix')

    # Node
    pcd_to_grid_node = Node(
        package='fast_lio',
        executable='pcd_to_grid',
        name='pcd_to_grid',
        output='screen',
        parameters=[{
            'pcd_file': pcd_file,
            'resolution': resolution,
            'min_z': min_z,
            'max_z': max_z,
            'downsample_factor': downsample_factor,
            'padding': padding,
            'output_prefix': output_prefix,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_pcd_file_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_min_z_cmd)
    ld.add_action(declare_max_z_cmd)
    ld.add_action(declare_downsample_cmd)
    ld.add_action(declare_padding_cmd)
    ld.add_action(declare_output_prefix_cmd)

    ld.add_action(pcd_to_grid_node)

    return ld
