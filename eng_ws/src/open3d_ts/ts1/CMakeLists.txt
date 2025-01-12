cmake_minimum_required(VERSION 3.22)
project(o3d_ts)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(colored_icp_example src/color_icp_o3d.cpp)
target_link_libraries(colored_icp_example Open3D::Open3D)
