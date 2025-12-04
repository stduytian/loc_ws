cmake_minimum_required(VERSION 3.8)
project(point2d_loc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)        # 必须是17
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(pointcloud_to_laserscan src/pointcloud_to_laserscan.cpp)

ament_target_dependencies(pointcloud_to_laserscan
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  pcl_conversions
  tf2_sensor_msgs
  tf2_geometry_msgs
  nav_msgs
  OpenCV
)

add_executable(point_look src/point_look.cpp)
ament_target_dependencies(point_look
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  OpenCV
)


add_executable(livox_to_scan src/livox_to_scan.cpp)
ament_target_dependencies(livox_to_scan
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  OpenCV
)

target_link_libraries(pointcloud_to_laserscan yaml-cpp)

install(TARGETS
  pointcloud_to_laserscan
  point_look
  livox_to_scan
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY map/
  DESTINATION share/${PROJECT_NAME}/map
)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
