cmake_minimum_required(VERSION 3.8)
project(ros2_driver)

# 设置 ROS 2 和 C++ 的支持
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(ros2_driver src/ros2_driver.cpp)

# 链接 ROS 2 库
ament_target_dependencies(ros2_driver rclcpp)

install(TARGETS
  ros2_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
