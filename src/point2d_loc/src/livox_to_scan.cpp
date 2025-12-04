#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> // for doTransform
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudTFNode : public rclcpp::Node
{
public:
  PointCloudTFNode() : Node("pointcloud_to_laserscan"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // 参数设置
    this->declare_parameter<std::string>("target_frame", "laser_link");
    this->declare_parameter<std::string>("source_frame", "livox_frame");
    this->declare_parameter<std::string>("cloud_topic", "/livox/lidar");

    target_frame_ = this->get_parameter("target_frame").as_string();
    source_frame_ = this->get_parameter("source_frame").as_string();
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::SensorDataQoS(),
                                                                    std::bind(&PointCloudTFNode::cloudCallback, this, std::placeholders::_1));

    rclcpp::QoS scan_qos(rclcpp::KeepLast(10)); // 保持 History 为 KeepLast(10)
    scan_qos.best_effort();
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    try
    {
      // 获取TF变换（可加时间同步逻辑）
      transformStamped = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    // 变换点云
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    try
    {
      tf2::doTransform(*msg, transformed_cloud, transformStamped);
      transformed_cloud.header.frame_id = target_frame_;
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "PointCloud transform failed: %s", ex.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(transformed_cloud, pcl_cloud);

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = this->get_clock()->now();
    scan_msg.header.frame_id = "laser_link";  // 指定 frame_id 为 laser_link
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = 2 * M_PI / 1200.0;
    scan_msg.range_min = 0.1;
    scan_msg.range_max = 20.0;

    int num_readings = std::ceil((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
    scan_msg.ranges.assign(num_readings, scan_msg.range_max + 1.0);

    // 投影到 2D LaserScan
    for (const auto &p : pcl_cloud.points)
    {
      if (p.z < 0.2 || p.z > 0.6)
        continue;

      float range = std::sqrt(p.x * p.x + p.y * p.y);
      if (range < scan_msg.range_min || range > scan_msg.range_max)
        continue;

      float angle = std::atan2(p.y, p.x);
      if (angle < scan_msg.angle_min || angle > scan_msg.angle_max)
        continue;

      int index = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
      if (index >= 0 && index < num_readings)
      {
        if (range < scan_msg.ranges[index])
        {
          scan_msg.ranges[index] = range;
        }
      }
    }

    pub_->publish(scan_msg);
  }

  std::string target_frame_, source_frame_, cloud_topic_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudTFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}