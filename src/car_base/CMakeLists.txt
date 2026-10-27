cmake_minimum_required(VERSION 3.20)

project(car_base)

# Humble(Ubuntu 22.04) 自带 CMake 3.22，满足 3.20 的最低要求
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找ROS 2构建系统
find_package(ament_cmake REQUIRED)

# 查找节点使用的依赖库
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 将源文件编译为可执行程序car_base_node
add_executable(
  car_base_node
  src/car_base_node.cpp
)

# car_base_node.cpp 使用 std::clamp 等 C++17 特性
target_compile_features(
  car_base_node
  PRIVATE cxx_std_17
)

# 给car_base_node链接依赖库
# 消息包统一使用 ${包名_TARGETS}，Humble 各版本都能解析；
# rclcpp / tf2_ros 使用 Humble 提供的现代目标名
target_link_libraries(car_base_node PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
)

# 将程序安装到ROS 2能够找到的位置
install(
  TARGETS car_base_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
