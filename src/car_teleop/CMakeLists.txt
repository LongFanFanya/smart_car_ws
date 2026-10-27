cmake_minimum_required(VERSION 3.20)

project(car_teleop)

# Humble(Ubuntu 22.04) 自带 CMake 3.22，满足 3.20 的最低要求
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  keyboard_teleop_node
  src/keyboard_teleop_node.cpp
)

add_executable(
  joy_teleop_node
  src/joy_teleop_node.cpp
)

foreach(target IN ITEMS
    keyboard_teleop_node
    joy_teleop_node)

  # 两个节点都使用 C++17 特性
  target_compile_features(
    ${target}
    PRIVATE cxx_std_17
  )

  # 消息包统一使用 ${包名_TARGETS}，Humble 各版本都能解析
  target_link_libraries(
    ${target}
    PRIVATE
    rclcpp::rclcpp
    ${geometry_msgs_TARGETS}
  )

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(
      ${target}
      PRIVATE -Wall -Wextra -Wpedantic
    )
  endif()

endforeach()

target_link_libraries(
  joy_teleop_node
  PRIVATE
  ${sensor_msgs_TARGETS}
)

install(
  TARGETS
    keyboard_teleop_node
    joy_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
