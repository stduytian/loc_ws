#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 用于四元数处理
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>

class SimpleNavigator : public rclcpp::Node {
public:
  SimpleNavigator() : Node("simple_navigator_cpp") {
    // 参数声明
    this->declare_parameter("linear_speed", 0.2);
    this->declare_parameter("angular_speed", 0.5);
    this->declare_parameter("distance_threshold", 0.2);
    this->declare_parameter("angle_threshold", 0.1);
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();
    distance_threshold_ = this->get_parameter("distance_threshold").as_double();
    angle_threshold_ = this->get_parameter("angle_threshold").as_double();

    // 订阅当前位置 (从Fast-LIO)
    current_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&SimpleNavigator::poseCallback, this, std::placeholders::_1));
    
    // 订阅目标点 (从RViz 2D Nav Goal)
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&SimpleNavigator::goalCallback, this, std::placeholders::_1));
    
    // 发布cmd_vel
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    // 控制循环定时器 (10Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&SimpleNavigator::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "Simple Navigator  started. Click 2D Nav Goal in RViz to set target.");
  }

private:
  void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose_ = *msg;
    RCLCPP_INFO(this->get_logger(), "New goal received at (%.2f, %.2f)",
                msg->pose.position.x, msg->pose.position.y);
  }

  void controlLoop() {
    if (!current_pose_.header.stamp.sec || !goal_pose_.header.stamp.sec) {
        std::cout<<current_pose_.header.stamp.sec<<" "<<goal_pose_.header.stamp.sec<<std::endl;
      return;  // 无有效位姿
    }

    // 计算距离
    double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
    double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 当前yaw (quaternion to euler)
    tf2::Quaternion q(current_pose_.pose.orientation.x,
                      current_pose_.pose.orientation.y,
                      current_pose_.pose.orientation.z,
                      current_pose_.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double current_yaw = yaw;

    // 目标yaw
    double goal_yaw = std::atan2(dy, dx);
    double angle_error = goal_yaw - current_yaw;
    // 归一化到[-pi, pi]
    angle_error = std::fmod(angle_error + M_PI, 2 * M_PI) - M_PI;

    auto twist = geometry_msgs::msg::Twist();

    if (distance > distance_threshold_) {
      if (std::abs(angle_error) > angle_threshold_) {
        // 先转向
        twist.angular.z = angular_speed_ * (angle_error > 0 ? 1.0 : -1.0);
        twist.linear.x = 0.0;
      } else {
        // 转向完成，前进
        twist.linear.x = linear_speed_;
        twist.angular.z = 0.0;
      }
    } else {
      // 到达，停止
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      goal_pose_.header.stamp.sec = 0;  // 重置目标
    }

    cmd_pub_->publish(twist);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  double linear_speed_, angular_speed_, distance_threshold_, angle_threshold_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNavigator>());
  rclcpp::shutdown();
  return 0;
}