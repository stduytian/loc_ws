#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudToLaserScanNode : public rclcpp::Node
{
public:
  PointCloudToLaserScanNode() : Node("point_look")
  {
    // 参数设置
    this->declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");

    cloud_topic_ = this->get_parameter("cloud_topic").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::SensorDataQoS(),
                                                                    std::bind(&PointCloudToLaserScanNode::cloudCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_body", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 假设点云已在目标frame中，直接使用
    sensor_msgs::msg::PointCloud2 transformed_cloud = *msg;
    transformed_cloud.header.frame_id = "p_body";

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud; 
    pcl::fromROSMsg(transformed_cloud, pcl_cloud);

    sensor_msgs::msg::LaserScan scan_msg; 
    scan_msg.header.stamp = transformed_cloud.header.stamp;
    scan_msg.header.frame_id = "p_body";
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = 2 * M_PI / 1200.0;
    scan_msg.range_min = 0.1;
    scan_msg.range_max = 20.0;

    int num_readings = std::ceil((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
    scan_msg.ranges.assign(num_readings, scan_msg.range_max + 1.0);

    for (const auto &p : pcl_cloud.points)
    {
      if (p.z < -0.1 || p.z > 0.1)
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
          scan_msg.ranges[index] = range;
      }
    }

    pub_->publish(scan_msg);
  }

  std::string cloud_topic_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudToLaserScanNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}