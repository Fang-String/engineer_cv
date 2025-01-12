#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> // 确保路径正确
#include <open3d/Open3D.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// ROS 2 回调函数，用于处理 PointCloud2 消息
void PointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 创建一个 PCL 点云对象
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    // 将 ROS PointCloud2 转换为 PCL 点云
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // 创建 Open3D 点云对象
    open3d::geometry::PointCloud o3d_cloud;

    // 将 PCL 点云中的数据转移到 Open3D 点云中
    for (const auto& point : pcl_cloud->points) {
        // PCL 点云中的颜色存储在 RGB 格式中，需要分离成 R, G, B 以便给 Open3D 点云
        o3d_cloud.points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));

        // Open3D 点云中的颜色范围为 [0, 1]，而 PCL 中是 [0, 255]，因此需要除以 255
        o3d_cloud.colors_.push_back(Eigen::Vector3d(point.r / 255.0, point.g / 255.0, point.b / 255.0));
    }

    // 打印转换后的 Open3D 点云的大小
    std::cout << "Converted to Open3D PointCloud, num points: " << o3d_cloud.points_.size() << std::endl;

    // 可选：可视化转换后的点云
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.push_back(std::make_shared<open3d::geometry::PointCloud>(o3d_cloud));
    open3d::visualization::DrawGeometries(geometries, "Converted PointCloud");
}

int main(int argc, char** argv) {
    // 初始化 ROS 2 节点
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pointcloud2_to_open3d");

    // 订阅 ROS 中的 PointCloud2 消息
    auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/mine_cloud", 10, PointCloud2Callback);

    // ROS 2 主循环
    rclcpp::spin(node);

    // 关闭 ROS 2 节点
    rclcpp::shutdown();
    return 0;
}