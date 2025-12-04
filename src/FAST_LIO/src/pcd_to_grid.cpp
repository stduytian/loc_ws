#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <fstream>
#include <limits>
#include <vector>
#include <string>
#include <cmath>

using std::string;

class PcdToGrid : public rclcpp::Node
{
public:
  PcdToGrid() : Node("pcd_to_grid")
  {
    this->declare_parameter<string>("pcd_file", "");
    this->declare_parameter<double>("resolution", 0.05);
    this->declare_parameter<double>("min_z", 0);
    this->declare_parameter<double>("max_z", 0.2);
    this->declare_parameter<int>("downsample_factor", 1);
    this->declare_parameter<double>("padding", 1.0);
    this->declare_parameter<string>("output_prefix", "map");

    pcd_file_ = this->get_parameter("pcd_file").as_string();
    resolution_ = this->get_parameter("resolution").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();
    RCLCPP_INFO(this->get_logger(), "min_z: %.2f, max_z: %.2f", min_z_, max_z_);
    downsample_factor_ = this->get_parameter("downsample_factor").as_int();
    padding_ = this->get_parameter("padding").as_double();
    output_prefix_ = this->get_parameter("output_prefix").as_string();

    if (pcd_file_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'pcd_file' is empty. Provide a path to a PCD file.");
      return;
    }

    if (resolution_ <= 0)
      resolution_ = 0.05;

    RCLCPP_INFO(this->get_logger(), "Loading PCD: %s", pcd_file_.c_str());
    process();
    RCLCPP_INFO(this->get_logger(), "Done. Wrote %s.pgm and %s.yaml", output_prefix_.c_str(), output_prefix_.c_str());
  }

private:
  void process()
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *cloud) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_file_.c_str());
      return;
    }

    // find bounds of points within Z range
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    size_t count = 0;
    for (size_t i = 0; i < cloud->points.size(); i += std::max(1, downsample_factor_))
    {
      const auto &p = cloud->points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      if (p.z < min_z_ || p.z > max_z_)
        continue;
      min_x = std::min(min_x, static_cast<double>(p.x));
      max_x = std::max(max_x, static_cast<double>(p.x));
      min_y = std::min(min_y, static_cast<double>(p.y));
      max_y = std::max(max_y, static_cast<double>(p.y));
      ++count;
    }

    if (count == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "No points in specified z range [%.2f, %.2f]. Nothing to project.", min_z_, max_z_);
      return;
    }

    // add padding
    min_x -= padding_;
    min_y -= padding_;
    max_x += padding_;
    max_y += padding_;

    int width = static_cast<int>(std::ceil((max_x - min_x) / resolution_));
    int height = static_cast<int>(std::ceil((max_y - min_y) / resolution_));

    if (width <= 0 || height <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Computed invalid map size: %d x %d", width, height);
      return;
    }

    // safety limit to avoid insane maps
    const int MAX_DIM = 20000; // prevent huge allocation
    if (width > MAX_DIM || height > MAX_DIM)
    {
      RCLCPP_ERROR(this->get_logger(), "Map too large (%d x %d). Increase resolution or reduce padding/pcd area.", width, height);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Map size: %d x %d, resolution=%.3f, points=%zu", width, height, resolution_, count);

    // initialize image with free value (254). Use map_server conventions: occupied=0, free=254, unknown=205
    std::vector<uint8_t> image(width * height, 254);

    // mark occupied cells
    for (size_t i = 0; i < cloud->points.size(); i += std::max(1, downsample_factor_))
    {
      const auto &p = cloud->points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        continue;
      if (p.z < min_z_ || p.z > max_z_)
        continue;
      int ix = static_cast<int>(std::floor((p.x - min_x) / resolution_));
      int iy = static_cast<int>(std::floor((p.y - min_y) / resolution_));
      if (ix < 0 || ix >= width || iy < 0 || iy >= height)
        continue;
      // image rows are written from top to bottom in PGM -> compute row index accordingly
      int row = height - 1 - iy;
      int idx = row * width + ix;
      image[idx] = 0; // occupied
    }

    // write PGM (binary P5)
    string pgm_file = output_prefix_ + ".pgm";
    std::ofstream ofs(pgm_file, std::ios::binary);
    if (!ofs)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", pgm_file.c_str());
      return;
    }
    ofs << "P5\n" << width << " " << height << "\n255\n";
    ofs.write(reinterpret_cast<const char *>(image.data()), image.size());
    ofs.close();

    // write YAML
    string yaml_file = output_prefix_ + ".yaml";
    std::ofstream yfs(yaml_file);
    if (!yfs)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s for writing", yaml_file.c_str());
      return;
    }
    // origin: [x, y, yaw] where x,y are bottom-left
    yfs << "image: " << pgm_file << "\n";
    yfs << "resolution: " << resolution_ << "\n";
    yfs << "origin: [" << min_x << ", " << min_y << ", 0.0]\n";
    yfs << "negate: 0\n";
    yfs << "occupied_thresh: 0.65\n";
    yfs << "free_thresh: 0.196\n";
    yfs.close();
  }

  // params
  string pcd_file_;
  double resolution_ = 0.05;
  double min_z_ = 0;
  double max_z_ = 0;
  int downsample_factor_ = 1;
  double padding_ = 1.0;
  string output_prefix_ = "map";
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PcdToGrid>();
  rclcpp::shutdown();
  return 0;
}
