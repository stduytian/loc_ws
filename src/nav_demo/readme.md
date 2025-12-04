## rviz2
2D Goal Pose 发布的话题名为：/move_base_simple/goal ？ 查看发现是/goal_pose
Type: geometry_msgs/msg/PoseStamped

2D Pose Estimate 发布的话题名为：/initialpose 
Type: geometry_msgs/msg/PoseWithCovarianceStamped

fast_lio的里程计/Odometry or /odom  
Type: nav_msgs/msg/Odometry

接收消息赋值的时候，注意消息话题和类型匹配。

## scout_mini
遥控器 SWB档位 在上档时，小车为can通讯控制模式，通过接收/cmd_vel消息实现移动。
Type: geometry_msgs/msg/Twist

在中档时，通过遥控器手动操作实现移动。

## 定位导航流程
1. 启动雷达驱动;
ros2 launch livox_ros_driver2 msg_MID360_launch.py

2. 启动2d点云icp配准初始位姿;
ros2 launch point2d_loc pointcloud_to_laserscan_launch.py

3. 启动fastlio接受初始位姿并开始输出里程计信息;
ros2 launch fast_lio mapping.launch.py

（rviz里面给出初始姿态估计）

4. 启动小车驱动;
sudo ip link set can0 up type can bitrate 500000
ros2 launch scout_base scout_mini_base.launch.py 

5. 启动nav2导航。
ros2 launch nav_demo nav2_odom_navigation.launch.py

(rviz里面给出导航点)
小车架子长120cm，robot_radius:0.60