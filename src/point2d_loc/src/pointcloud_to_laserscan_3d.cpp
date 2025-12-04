#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>  // for doTransform
// #include <cloud_scan/ndt_omp.h>
struct MapMeta {
  std::string image_file;
  double resolution = 0.05;
  std::vector<double> origin = {0.0, 0.0, 0.0};
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  int negate = 0;
};

class PointCloudTFNode : public rclcpp::Node {
 public:
  PointCloudTFNode()
      : Node("pointcloud_to_laserscan"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // 参数设置
    this->declare_parameter<std::string>("target_frame", "laser");
    this->declare_parameter<std::string>("source_frame", "livox_frame");
    this->declare_parameter<std::string>("cloud_topic", "/livox/lidar");
    this->declare_parameter<std::string>(
        "pgm_path",
        "/home/dongkuo/lt_ws/loc_ws/src/point2d_loc/map/");
    this->declare_parameter<std::string>("yaml_file", "");
    this->declare_parameter<std::string>("image_file", "");

    target_frame_ = this->get_parameter("target_frame").as_string();
    source_frame_ = this->get_parameter("source_frame").as_string();
    cloud_topic_ = this->get_parameter("cloud_topic").as_string();
    pgm_path_ = this->get_parameter("pgm_path").as_string();
    yaml_file_param_ = this->get_parameter("yaml_file").as_string();
    image_file_param_ = this->get_parameter("image_file").as_string();

    scan_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    lidar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map_3dcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    std::string pcd_file = pgm_path_ + "test.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *map_3dcloud_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "加载 PCD 文件失败: %s",
                   pcd_file.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "成功加载 PCD: %s 点数=%zu",
                  pcd_file.c_str(), map_3dcloud_->size());
    }
    
    pcl::toROSMsg(*map_3dcloud_, map_cloud_msg_);
    map_cloud_msg_.header.frame_id = "map";
    map_cloud_msg_.header.stamp = this->get_clock()->now();

    std::string yaml_file = !yaml_file_param_.empty()
                                ? yaml_file_param_
                                : (pgm_path_ + "neibor_office2.yaml");
    std::string image_file = !image_file_param_.empty()
                                 ? image_file_param_
                                 : (pgm_path_ + "neibor_office2.pgm");
    map_msg_ = loadMap(yaml_file, 0.05, image_file);
    map_msg_.header.frame_id = "map";
    map_msg_.header.stamp = this->get_clock()->now();
    cloud_msg_ = mapToPointCloud(map_msg_);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PointCloudTFNode::cloudCallback, this,
                  std::placeholders::_1));
    init_pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", rclcpp::SensorDataQoS(),
        std::bind(&PointCloudTFNode::poseCallback, this,
                  std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    map_3dcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_3dcloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    align_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/align_cloud", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    align_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/align_flag", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/slam_init_pose", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    timer_2d_map_ = this->create_wall_timer(
        std::chrono::seconds(1), [this]() { map_pub_->publish(map_msg_); });
    timer_3d_map_ = this->create_wall_timer(
    std::chrono::seconds(1), [this]() {
        map_cloud_msg_.header.stamp = this->get_clock()->now();
        map_cloud_pub_->publish(map_cloud_msg_);
    });
  }
 private:
  std::string yaml_file_param_, image_file_param_;

  std::string target_frame_, source_frame_, cloud_topic_, pgm_path_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      init_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_3dcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr align_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr align_flag_pub_;
  rclcpp::TimerBase::SharedPtr timer_2d_map_;
  rclcpp::TimerBase::SharedPtr timer_3d_map_;

  nav_msgs::msg::OccupancyGrid map_msg_;
  sensor_msgs::msg::PointCloud2 map_cloud_msg_;
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_3dcloud_;
  MapMeta meta;
 private:
  double computeIcpFitnessScore(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
    if (!source || !target || source->empty() || target->empty()) {
      return std::numeric_limits<double>::max();
    }

    // KdTree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target);

    const float match_threshold = 0.5f;
    const float match_threshold_sq = match_threshold * match_threshold;  // 0.01

    double total_error = 0.0;
    int valid_count = 0;

    for (const auto& pt : source->points) {
      std::vector<int> idx(1);
      std::vector<float> dist_sq(1);

      if (kdtree.nearestKSearch(pt, 1, idx, dist_sq) > 0) {
        if (dist_sq[0] < match_threshold_sq) {
          total_error += dist_sq[0];
          valid_count++;
        }
      }
    }

    if (valid_count < 10) {
      return std::numeric_limits<double>::max();
    }

    return static_cast<double>(valid_count) / source->points.size();
  }

  void poseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double roll, pitch, yaw;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(this->get_logger(), "2D Pose Estimate: x=%.2f y=%.2f yaw=%.2f",
                x, y, yaw);
    if (!lidar_cloud_ || lidar_cloud_->empty() || !map_3dcloud_ ||
        map_3dcloud_->empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Scan cloud or map cloud is empty, skip ICP.");
      return;
    }

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess(0, 0) = std::cos(yaw);
    init_guess(0, 1) = -std::sin(yaw);
    init_guess(1, 0) = std::sin(yaw);
    init_guess(1, 1) = std::cos(yaw);
    init_guess(0, 3) = x;
    init_guess(1, 3) = y;

    auto origin_cloud = lidar_cloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // pcl::PointCloud<pcl::PointXYZ> aligned;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

    // 对源点云降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_origin(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f); // 根据点云密度调整
    voxel_filter.setInputCloud(origin_cloud);
    voxel_filter.filter(*downsampled_origin);
    // 对目标点云降采样  
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 根据点云密度调整
    voxel_filter.setInputCloud(map_3dcloud_);
    voxel_filter.filter(*downsampled_map);
    for (int i = 0; i < 10; i++) {
      icp.setInputSource(downsampled_origin);
      icp.setInputTarget(downsampled_map);
      icp.setMaximumIterations(20);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-6);
      icp.setMaxCorrespondenceDistance(0.1);
      
      icp.align(*aligned, init_guess);
      init_guess = icp.getFinalTransformation();
      // double match_score = computeIcpFitnessScore(
      //     std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(aligned),
      //     map_cloud_);
      double match_score = computeIcpFitnessScore( aligned, map_cloud_);
      RCLCPP_INFO(
          this->get_logger(),
          "match score:%.6f",match_score
         );
      if (match_score > 0.9) {
        break;
      }
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_msg.header.frame_id = "map";
      aligned_msg.header.stamp = this->get_clock()->now();
      align_cloud_pub_->publish(aligned_msg);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_ndt(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(1e-8);
    ndt.setStepSize(0.1);                  // 梯度下降步长
    ndt.setResolution(0.25);                // NDT 栅格分辨率
    ndt.setMaximumIterations(50);          // 最大迭代次数
    ndt.setInputSource(aligned);
    ndt.setInputTarget(downsampled_map);
    ndt.align(*aligned_ndt, init_guess);
    sensor_msgs::msg::PointCloud2 aligned_msgg;
      pcl::toROSMsg(*aligned_ndt, aligned_msgg);
      aligned_msgg.header.frame_id = "map";
      aligned_msgg.header.stamp = this->get_clock()->now();
      align_cloud_pub_->publish(aligned_msgg);

    if (ndt.hasConverged()) {
      Eigen::Matrix4f final_tf = ndt.getFinalTransformation();

      double aligned_x = final_tf(0, 3);
      double aligned_y = final_tf(1, 3);
      double aligned_yaw = std::atan2(final_tf(1, 0), final_tf(0, 0));

      RCLCPP_INFO(
          this->get_logger(),
          "ICP converged: score=%.6f, Final Pose: x=%.2f y=%.2f yaw=%.2f deg",
          ndt.getFitnessScore(), aligned_x, aligned_y,
          aligned_yaw * 180.0 / M_PI);
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
    } else {
      RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
    }
  }
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
      // 获取TF变换（可加时间同步逻辑）
      transformStamped = tf_buffer_.lookupTransform(
          target_frame_, msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    // 变换点云
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    try {
      tf2::doTransform(*msg, transformed_cloud, transformStamped);
      transformed_cloud.header.frame_id = target_frame_;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "PointCloud transform failed: %s",
                   ex.what());
    }

    lidar_cloud_ -> clear();
    pcl::fromROSMsg(transformed_cloud, *lidar_cloud_);

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = transformed_cloud.header.stamp;
    scan_msg.header.frame_id = "p_body";
    scan_msg.angle_min = -M_PI;
    scan_msg.angle_max = M_PI;
    scan_msg.angle_increment = 2 * M_PI / 1200.0;
    scan_msg.range_min = 0.1;
    scan_msg.range_max = 20.0;

    int num_readings = std::ceil((scan_msg.angle_max - scan_msg.angle_min) /
                                 scan_msg.angle_increment);
    scan_msg.ranges.assign(num_readings, scan_msg.range_max + 1.0);
    for (const auto& p : lidar_cloud_->points) {
      if (p.z < -0.1 || p.z > 0.1) continue;

      float range = std::sqrt(p.x * p.x + p.y * p.y);
      if (range < scan_msg.range_min || range > scan_msg.range_max) continue;

      float angle = std::atan2(p.y, p.x);
      if (angle < scan_msg.angle_min || angle > scan_msg.angle_max) continue;

      int index = static_cast<int>((angle - scan_msg.angle_min) /
                                   scan_msg.angle_increment);
      if (index >= 0 && index < num_readings) {
        if (range < scan_msg.ranges[index]) scan_msg.ranges[index] = range;
      }
    }

    pub_->publish(scan_msg);

    pcl::PointCloud<pcl::PointXYZ> scan_as_cloud;
    for (size_t i = 0; i < scan_msg.ranges.size(); ++i) {
      float range = scan_msg.ranges[i];

      // 排除无效点
      if (range > scan_msg.range_max) continue;

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

  nav_msgs::msg::OccupancyGrid loadMap(
      const std::string& pgm_file, double resolution,
      const std::string& override_image_file = "") {
    YAML::Node config = YAML::LoadFile(pgm_file);
    meta.image_file = config["image"].as<std::string>();
    std::string image_file = !override_image_file.empty()
                                 ? override_image_file
                                 : (pgm_path_ + meta.image_file);

    meta.resolution = config["resolution"].as<double>();
    if (config["origin"]) {
      auto origin_node = config["origin"];
      for (size_t i = 0; i < origin_node.size(); ++i)
        meta.origin[i] = origin_node[i].as<double>();
    }

    if (config["occupied_thresh"])
      meta.occupied_thresh = config["occupied_thresh"].as<double>();
    if (config["free_thresh"])
      meta.free_thresh = config["free_thresh"].as<double>();
    if (config["negate"]) meta.negate = config["negate"].as<int>();

    cv::Mat img = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PGM file: %s",
                   pgm_file.c_str());
      throw std::runtime_error("Failed to load PGM file");
    }
    if (meta.negate) img = 255 - img;

    cv::flip(img, img, 0);  // 翻转Y轴

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
    for (int y = 0; y < img.rows; ++y) {
      for (int x = 0; x < img.cols; ++x) {
        uint8_t val = img.at<uint8_t>(y, x);
        double occ_prob = (255 - val) / 255.0;

        if (occ_prob > meta.occupied_thresh)
          map.data.push_back(100);
        else if (occ_prob < meta.free_thresh)
          map.data.push_back(0);
        else
          map.data.push_back(-1);  // 未知
      }
    }

    return map;
  }

  sensor_msgs::msg::PointCloud2 mapToPointCloud(
      const nav_msgs::msg::OccupancyGrid& map_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = map_msg.header.frame_id;

    const double resolution = map_msg.info.resolution;
    const double origin_x = map_msg.info.origin.position.x;
    const double origin_y = map_msg.info.origin.position.y;

    const int width = map_msg.info.width;
    const int height = map_msg.info.height;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = y * width + x;
        int value = map_msg.data[idx];
        if (value == 100)  // 只处理障碍物
        {
          pcl::PointXYZ pt;
          pt.x = origin_x + (x + 0.5) * resolution;
          pt.y = origin_y + (y + 0.5) * resolution;
          pt.z = 0.0;  // 可自定义高度
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
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudTFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}