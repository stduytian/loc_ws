cmake_minimum_required(VERSION 3.8)
project(nav_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # 用于tf2转换
find_package(nav_msgs REQUIRED)

# 包含目录
include_directories(include)

# 可执行文件
add_executable(simple_navigator src/simple_navigator.cpp)
ament_target_dependencies(simple_navigator
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  nav_msgs
)

# 安装
install(TARGETS simple_navigator
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 与 config 目录，供 ros2 launch 查找
install(DIRECTORY launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()