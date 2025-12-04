import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml'
        ),
        description='Nav2 参数文件（使用 map 系导航）'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='/home/dongkuo/lt_ws/loc_ws/map/neibor_office3.yaml',
        description='静态地图 YAML 文件'
    )

    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml}]
    )

    # Planner
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # Controller
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    # Waypoint Follower（可选）
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )

    smoother_server = Node(
    package='nav2_smoother',
    executable='smoother_server',
    name='smoother_server',
    output='screen',
    parameters=[params_file]
    )

    # velocity_smoother = Node(
    # package='nav2_util',
    # executable='velocity_smoother',
    # name='velocity_smoother',
    # output='screen',
    # parameters=[params_file]
    # )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server',
                'waypoint_follower',
                'smoother_server',  # 新增
                # 'velocity_smoother'
            ]
        }]
    )

    return LaunchDescription([
        declare_params,
        declare_map,
        map_server,
        planner_server,
        controller_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        smoother_server, 
        # velocity_smoother
    ])


