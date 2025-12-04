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
    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_frame_to_laser',
        arguments=['0', '0', '0', '0', '-0.244781', '0', 'livox_frame', 'laser'],
        output='screen'
    )

    map_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0.26763', '0', '0', '0', 'map', 'camera_init'],
        output='screen'
    )

    body_to_p_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_p_body',
        arguments=['0', '0', '0', '0', '-0.244781', '0', 'body', 'p_body'],
        output='screen'
    )

    p_body_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='p_body_to_base_footprint',
        arguments=['-0.55851', '0', '-0.38617', '0', '0', '0', 'p_body', 'base_footprint'],
        output='screen'
    )

    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.11854', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )
    
    pointcloud_to_laserscan = Node(
        package='point2d_loc',
        executable='pointcloud_to_laserscan',
        name='pointcloud_to_laserscan',
        output='screen', 
        parameters=[{
            'yaml_file': '/home/dongkuo/lt_ws/loc_ws/map/new.yaml',
            'image_file': '/home/dongkuo/lt_ws/loc_ws/map/new.pgm'
        }]
    )

    share_dir = get_package_share_directory('point2d_loc')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'loc_map.rviz')
    # 如果安装路径不存在，尝试使用源文件路径
    if not os.path.exists(rviz_config_file):
        # 回退到源文件路径
        pkg_dir = os.path.join(os.path.dirname(__file__), '..')
        rviz_config_file = os.path.join(pkg_dir, 'rviz', 'loc_map.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_rviz,
        static_tf_node, 
        map_to_camera_init,
        pointcloud_to_laserscan,
        body_to_p_body,
        p_body_to_base_footprint,
        base_footprint_to_base_link,
        rviz_node
    ])
