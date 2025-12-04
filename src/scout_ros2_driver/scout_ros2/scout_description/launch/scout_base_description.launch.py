import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    model_name = 'scout_mini.xacro' # 机器人模型（Xacro 是 URDF 的 XML 宏扩展）
    # model_path = os.path.join(get_package_share_directory('scout_description'), "urdf", model_name)
    # print(model_path)
    #动态执行 xacro 命令，生成 URDF 字符串（作为 robot_description 参数传入节点）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",  # /opt/ros/humble/bin/xacro
        PathJoinSubstitution(
            [FindPackageShare("scout_description"), "urdf", model_name]
        ),
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation clock if true'),

        launch.actions.LogInfo(msg='use_sim_time: '),
        launch.actions.LogInfo(msg=launch.substitutions.LaunchConfiguration('use_sim_time')),
        
        # 读取 URDF，计算并广播所有链接间的 TF 变换
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'robot_description':robot_description_content
            }]),
    ])
