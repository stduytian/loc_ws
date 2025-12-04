#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp> // for doTransform
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <yaml-cpp/yaml.h>

#include <nav_msgs/msg/occupancy_grid.hpp>


struct MapMeta
{
    std::string image_file;
    double resolution = 0.05;
    std::vector<double> origin = {0.0, 0.0, 0.0};
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
    int negate = 0;
};

class PointCloudTFNode : public rclcpp::Node
{
public:
  PointCloudTFNode() : Node("pointcloud_to_laserscan"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    // 参数设置 
    this->declare_parameter<std::string>("target_frame", "laser");
    this->declare_parameter<std::string>("source_frame", "livox_frame");
    this->declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    this->declare_parameter<std::string>("pgm_path", "/home/dongkuo/lt_ws/automove_ws/src/pointcloud_to_laserscan/map/");

    target_frame_ = this->get_parameter("target_frame").as_string();
    source_frame_ = this->get_parameter("source_frame").as_string();
    cloud_topic_  = this->get_parameter("cloud_topic").as_string();
    pgm_path_ = this->get_parameter("pgm_path").as_string();

    scan_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    std::string yaml_file = pgm_path_+"office2.yaml";
    map_msg_ = loadMap(yaml_file,0.05);
    cloud_msg_ = mapToPointCloud(map_msg_);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&PointCloudTFNode::cloudCallback, this, std::placeholders::_1));
    init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",  rclcpp::SensorDataQoS(),std::bind(&PointCloudTFNode::poseCallback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    align_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/align_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    align_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("/align_glag", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam_init_pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    timer_ = this->create_wall_timer(std::chrono::seconds(1),[this]() { map_cloud_pub_->publish(cloud_msg_); });
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double roll, pitch, yaw;
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(this->get_logger(), "2D Pose Estimate: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);
    if (!scan_cloud_ || scan_cloud_->empty() || !map_cloud_ || map_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "Scan cloud or map cloud is empty, skip ICP.");
        return;
    }

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess(0, 0) = std::cos(yaw);
    init_guess(0, 1) = -std::sin(yaw);
    init_guess(1, 0) = std::sin(yaw);
    init_guess(1, 1) = std::cos(yaw);
    init_guess(0, 3) = x;
    init_guess(1, 3) = y;

    auto origin_cloud = scan_cloud_;

    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(origin_cloud);
    icp.setInputTarget(map_cloud_);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setMaxCorrespondenceDistance(0.5);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, init_guess);

    if (icp.hasConverged())
    {
        Eigen::Matrix4f final_tf = icp.getFinalTransformation();

        double aligned_x = final_tf(0, 3);
        double aligned_y = final_tf(1, 3);
        double aligned_yaw = std::atan2(final_tf(1, 0), final_tf(0, 0));

        RCLCPP_INFO(this->get_logger(),
                    "ICP converged: score=%.6f, Final Pose: x=%.2f y=%.2f yaw=%.2f deg",
                    icp.getFitnessScore(),
                    aligned_x, aligned_y, aligned_yaw * 180.0 / M_PI);
        // ✅ 在回调里循环发布（非阻塞）
        std::thread([this, aligned]() {
            rclcpp::Rate rate(1.0);  // 每秒 2 次
            for (int i = 0; rclcpp::ok(); ++i)
            {
                sensor_msgs::msg::PointCloud2 aligned_msg;
                pcl::toROSMsg(aligned, aligned_msg);
                aligned_msg.header.frame_id = "map";
                aligned_msg.header.stamp = this->get_clock()->now();
                align_cloud_pub_->publish(aligned_msg);

                rate.sleep();
            }
        }).detach(); // 分离线程，不阻塞主线程
        std_msgs::msg::Bool flag_msg;
        flag_msg.data = true;
        align_flag_pub_->publish(flag_msg);

        geometry_msgs::msg::PoseStamped geo_msg;
        geo_msg.pose.position.x = aligned_x;
        geo_msg.pose.position.y = aligned_y;
        double roll = 0.0;
        double pitch = 0.0;

        // 构造四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, aligned_yaw);

        // 转成 ROS2 的 geometry_msgs 格式
        geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
        geo_msg.pose.orientation = q_msg;
        local_pos_pub_->publish(geo_msg);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
    }
  }
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
    }

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(transformed_cloud, pcl_cloud);

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = transformed_cloud.header.stamp;
    scan_msg.header.frame_id = target_frame_;
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = 2*M_PI/1200.0;
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

    pcl::PointCloud<pcl::PointXYZ> scan_as_cloud;
    for (size_t i = 0; i < scan_msg.ranges.size(); ++i)
    {
        float range = scan_msg.ranges[i];

        // 排除无效点
        if (range > scan_msg.range_max)
            continue;

        float angle = scan_msg.angle_min + i * scan_msg.angle_increment;

        pcl::PointXYZ pt;
        pt.x = range * std::cos(angle);
        pt.y = range * std::sin(angle);
        pt.z = 0.0;  // 可根据需要设置高度
        scan_as_cloud.points.push_back(pt);
    }

    scan_as_cloud.width = scan_as_cloud.points.size();
    scan_as_cloud.height = 1;
    scan_as_cloud.is_dense = true;
    scan_cloud_->clear();
    *scan_cloud_ = scan_as_cloud;
  }

  nav_msgs::msg::OccupancyGrid loadMap(const std::string &pgm_file, double resolution)
  {
    YAML::Node config = YAML::LoadFile(pgm_file);
    meta.image_file = config["image"].as<std::string>();
    std::string image_file = pgm_path_+"office2.pgm";
    meta.resolution = config["resolution"].as<double>();
    if (config["origin"])
    {
      auto origin_node = config["origin"];
      for (size_t i = 0; i < origin_node.size(); ++i)
        meta.origin[i] = origin_node[i].as<double>();
    }

    if (config["occupied_thresh"])
      meta.occupied_thresh = config["occupied_thresh"].as<double>();
    if (config["free_thresh"])
      meta.free_thresh = config["free_thresh"].as<double>();
    if (config["negate"])
      meta.negate = config["negate"].as<int>();

    cv::Mat img = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PGM file: %s", pgm_file.c_str());
      throw std::runtime_error("Failed to load PGM file");
    }
    if (meta.negate)
      img = 255 - img;

    cv::flip(img, img, 0); // 翻转Y轴

    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.header.stamp = this->get_clock()->now();
    map.info.width = img.cols;
    map.info.height = img.rows;
    map.info.resolution = meta.resolution;
    map.info.origin.position.x = meta.origin[0];
    map.info.origin.position.y = meta.origin[1];
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;

    map.data.reserve(img.rows * img.cols);
    for (int y = 0; y < img.rows; ++y)
    {
      for (int x = 0; x < img.cols; ++x)
      {
        uint8_t val = img.at<uint8_t>(y, x);
        double occ_prob = (255 - val) / 255.0;

        if (occ_prob > meta.occupied_thresh)
          map.data.push_back(100);
        else if (occ_prob < meta.free_thresh)
          map.data.push_back(0);
        else
          map.data.push_back(-1); // 未知
      }
    }

    return map;
  }

  sensor_msgs::msg::PointCloud2 mapToPointCloud(const nav_msgs::msg::OccupancyGrid &map_msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = map_msg.header.frame_id;

    const double resolution = map_msg.info.resolution;
    const double origin_x = map_msg.info.origin.position.x;
    const double origin_y = map_msg.info.origin.position.y;

    const int width = map_msg.info.width;
    const int height = map_msg.info.height;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int idx = y * width + x;
            int value = map_msg.data[idx];
            if (value == 100) // 只处理障碍物
            {
                pcl::PointXYZ pt;
                pt.x = origin_x + (x + 0.5) * resolution;
                pt.y = origin_y + (y + 0.5) * resolution;
                pt.z = 0.0; // 可自定义高度
                cloud->points.push_back(pt);
            }
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    map_cloud_->clear();
    map_cloud_ = cloud;
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = this->get_clock()->now();
    return cloud_msg;
  }

  std::string target_frame_, source_frame_, cloud_topic_, pgm_path_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr align_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr align_flag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid map_msg_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  MapMeta meta;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudTFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
