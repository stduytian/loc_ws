#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PcdPublisher : public rclcpp::Node
{
public:
  PcdPublisher() : Node("pcd_publisher")
  {
    // 声明参数：PCD 文件路径
    this->declare_parameter<std::string>("pcd_file", "example.pcd");
    pcd_file_ = this->get_parameter("pcd_file").as_string();
    // 声明降采样因子参数：默认每100个点取1个
    this->declare_parameter<int>("downsample_factor", 100);
    downsample_factor_ = this->get_parameter("downsample_factor").as_int();

  // 创建发布者：使用 transient_local QoS（等价于 ROS1 的 latch），
  // 这样新订阅者（比如后来打开的 RViz）也能接收到最后一次发布的消息。
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local();
  qos.reliable();
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcd_cloud", qos);

    RCLCPP_INFO(this->get_logger(), "PCD Publisher started. Loading and publishing file once: %s", pcd_file_.c_str());

    // 立即加载并发布一次
    publish_cloud();
  }

private:
  void publish_cloud()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 加载 PCD 文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *cloud) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file_.c_str());
      return;
    }

    // 降采样：每 downsample_factor_ 个点取 1 个
    pcl::PointCloud<pcl::PointXYZ>::Ptr down_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (downsample_factor_ <= 1)
    {
      // 不降采样或无效值，直接复制
      *down_cloud = *cloud;
    }
    else
    {
      down_cloud->points.reserve((cloud->points.size() + downsample_factor_ - 1) / downsample_factor_);
      for (size_t i = 0; i < cloud->points.size(); i += static_cast<size_t>(downsample_factor_))
      {
        down_cloud->points.push_back(cloud->points[i]);
      }
      down_cloud->width = static_cast<uint32_t>(down_cloud->points.size());
      down_cloud->height = 1;
      down_cloud->is_dense = cloud->is_dense;
    }

    // 转换为 ROS 消息（使用降采样后的点云）
    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(*down_cloud, ros_cloud);

    // 设置消息头（帧 ID 和时间戳）
    ros_cloud.header.frame_id = "map";  // 可根据需要修改
    ros_cloud.header.stamp = this->get_clock()->now();

    // 发布
    publisher_->publish(ros_cloud);

    RCLCPP_INFO(this->get_logger(), "Published %zu points from PCD file (once). Original: %zu -> Downsampled: %zu (factor=%d)",
                down_cloud->size(), cloud->size(), down_cloud->size(), downsample_factor_);
  }

  std::string pcd_file_;
  int downsample_factor_ = 100;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcdPublisher>();
  rclcpp::spin(node);  // 保持节点运行，便于RViz订阅
  rclcpp::shutdown();
  return 0;
}