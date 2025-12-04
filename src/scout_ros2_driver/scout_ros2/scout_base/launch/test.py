#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleMoveDemo(Node):
    def __init__(self):
        super().__init__('simple_move_demo')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Demo 节点已启动，准备控制机器人...')
        
        # 等待 2 秒让机器人准备好
        time.sleep(2.0)
        
        # 前进 20cm (0.2m) @ 0.5 m/s → 时间 = 0.2 / 0.5 = 0.4s
        self.publish_velocity(linear_x=0.2)
        self.get_logger().info('前进 20cm...')
        time.sleep(2)
        
        # 停止
        self.publish_velocity(linear_x=0.0)
        self.get_logger().info('停止...')
        time.sleep(0.1)
        
        # 后退 20cm (0.2m) @ 0.5 m/s → 时间 = 0.2 / 0.5 = 0.4s
        self.publish_velocity(linear_x=-0.2)
        self.get_logger().info('后退 20cm...')
        time.sleep(1)
        
        # 最终停止
        self.publish_velocity(linear_x=0.0)
        self.get_logger().info('最终停止，demo 结束！')
        
        # 保持节点运行 1 秒后退出
        time.sleep(1.0)

    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z  # 默认为 0，无旋转
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleMoveDemo()
    
    # 运行节点（但由于是单次 demo，直接销毁）
    rclpy.spin_once(node, timeout_sec=0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
