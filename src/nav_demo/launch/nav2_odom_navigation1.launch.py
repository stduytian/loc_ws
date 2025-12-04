import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
        map_file=os.path.join(
                get_package_share_directory('nav_demo'),
                'maps',
                'neibor_office3.yaml'
                )
        nav_param_file=os.path.join(
                get_package_share_directory('nav_demo'),
                'config',
                'nav2_params.yaml'
                )
        nav2_launch_dir=os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',  
        )
        navgation_cmd=IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_dir + '/bringup_launch.py'),
                launch_arguments={
                        'map': map_file,
                        'use_sim_time': 'False',
                        'params_file': nav_param_file,
                        'localization': 'False'  # 新增：禁用 AMCL 和定位栈
                        }.items(),
        )
        rviz_file=os.path.join(
                get_package_share_directory('nav_demo'),
                'rviz',
                'navi.rviz')
        rviz_cmd=Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_file],
        )

        ld=LaunchDescription()
        ld.add_action(navgation_cmd)
        ld.add_action(rviz_cmd)
        return ld