# Nav2 导航配置说明

## 概述
本配置使用 fast_lio 的里程计进行定位（不使用 AMCL），配置了完整的 Nav2 导航功能。

## TF 树结构
```
map -> camera_init -> body -> p_body -> base_footprint -> base_link
```

## 配置要点

### 1. 坐标系配置
- **global_frame**: `map` - 全局地图坐标系
- **odom_frame**: `camera_init` - 里程计坐标系（fast_lio 发布）
- **robot_base_frame**: `base_link` - 机器人基座坐标系
- **sensor_frame**: `p_body` - 激光雷达坐标系

### 2. 话题配置
- **里程计话题**: `/odom` (frame_id: `camera_init`)
- **激光雷达话题**: `/scan` (frame_id: `p_body`)
- **速度控制话题**: `/cmd_vel`
- **地图文件**: `/home/dongkuo/lt_ws/loc_ws/map/office2.yaml`

### 3. 主要组件

#### 地图服务器 (map_server)
- 加载静态地图文件
- 提供全局地图信息

#### 全局代价地图 (global_costmap)
- 坐标系: `map`
- 基于静态地图 + 激光雷达障碍物
- 用于全局路径规划

#### 局部代价地图 (local_costmap)
- 坐标系: `camera_init` (odom frame)
- 滚动窗口模式 (8m x 8m)
- 基于激光雷达障碍物
- 用于局部避障

#### 规划器服务器 (planner_server)
- 使用 NavfnPlanner (A* 算法)
- 在 `map` 坐标系上进行全局路径规划

#### 控制器服务器 (controller_server)
- 使用 Regulated Pure Pursuit Controller
- 订阅 `/odom` 话题
- 发布 `/cmd_vel` 控制命令
- 使用局部代价地图进行碰撞检测

#### 行为树导航器 (bt_navigator)
- 协调导航行为
- 处理目标到达、路径重规划等

#### 恢复行为服务器 (behavior_server)
- 提供恢复行为：旋转、后退、等待

## 使用方法

### 启动导航系统
```bash
ros2 launch nav_demo nav2_odom_navigation.launch.py
```

### 发送导航目标
可以通过以下方式发送导航目标：

1. **使用 RViz2**:
   - 启动 RViz2
   - 使用 "2D Goal Pose" 工具在地图上点击目标点

2. **使用命令行**:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
{
  pose: {
    header: {
      frame_id: 'map'
    },
    pose: {
      position: {x: 1.0, y: 1.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

### 检查 TF 树
```bash
ros2 run tf2_tools view_frames
```

### 查看话题
```bash
# 查看所有话题
ros2 topic list

# 查看里程计信息
ros2 topic echo /odom

# 查看激光雷达数据
ros2 topic echo /scan

# 查看代价地图
ros2 topic echo /local_costmap/costmap
ros2 topic echo /global_costmap/costmap
```

## 注意事项

1. **TF 变换要求**:
   - 确保 fast_lio 正在发布 `map -> camera_init` 的 TF 变换
   - 确保 `camera_init -> body -> p_body -> base_footprint -> base_link` 的 TF 链完整

2. **地图文件**:
   - 确保地图文件路径正确
   - 地图分辨率: 0.05m
   - 地图格式: PGM + YAML

3. **激光雷达**:
   - 确保 `/scan` 话题正常发布
   - frame_id 必须为 `p_body`

4. **里程计**:
   - 确保 `/odom` 话题正常发布
   - frame_id 必须为 `camera_init`

## 参数调整

主要可调整参数位于 `nav2_params.yaml`:

- **控制器速度**: `controller_server.FollowPath.desired_linear_vel`
- **机器人半径**: `global_costmap.global_costmap.ros__parameters.robot_radius`
- **局部地图大小**: `local_costmap.local_costmap.ros__parameters.width/height`
- **膨胀半径**: `inflation_layer.inflation_radius`

## 故障排查

1. **无法规划路径**:
   - 检查 TF 树是否完整
   - 检查地图是否正确加载
   - 检查目标点是否在地图范围内

2. **机器人不移动**:
   - 检查 `/cmd_vel` 话题是否发布
   - 检查控制器是否正常启动
   - 检查是否有障碍物阻挡

3. **代价地图为空**:
   - 检查激光雷达数据是否正常
   - 检查 TF 变换是否正确
   - 检查 costmap 节点是否正常启动

