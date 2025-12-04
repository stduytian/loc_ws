from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld=LaunchDescription()
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_camera_init_tf',
        arguments=['0','0','0','0','0.261799','0','map','camera_init'],#0.261799 1.5708
        output='screen'
    )
    ld.add_action(static_tf)
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        ])
    )
    ld.add_action(livox_launch)
    fast_lio_launch = TimerAction(
        period=1.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('fast_lio'),
                        'launch',
                        'mapping.launch.py'
                    ])
                ])
            )
        ]
    )
    ld.add_action(fast_lio_launch)

    octomap_node=TimerAction(
        period=1.0,
        actions=[
            Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                remappings=[('cloud_in','/cloud_registered_body')],
                parameters=[{
                    'frame_id': 'map',
                    'resolution': 0.05,
                    'sensor_model/max_range': 60.0,
                    'pointcloud_min_z': -1.0,
                    'pointcloud_max_z': 2.0,
                    'occupancy_min_z': -0.30,
                    'occupancy_max_z': 0.30,
                    'ws_port': 9001,  # 新增：WS 端口参数
                    'publish_2d_map': True
               }]  
            )
        ]
    )
    ld.add_action(octomap_node)
    return ld