#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        // 获取 YAML 文件路径
        std::string yaml_file;
        this->declare_parameter<std::string>("camera_info_file", "");
        this->get_parameter("camera_info_file", yaml_file);

        if (yaml_file.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Camera info file parameter is empty!");
            return;
        }

        // 加载 YAML 文件
        if (!loadCameraInfo(yaml_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera info from %s", yaml_file.c_str());
            return;
        }

        // 示例：加载图像并进行去畸变操作
        cv::Mat input_image = cv::imread("input.jpg");
        if (input_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load input image!");
            return;
        }

        cv::Mat undistorted_image;
        cv::undistort(input_image, undistorted_image, camera_matrix_, dist_coeffs_);

        // 保存结果
        cv::imwrite("undistorted.jpg", undistorted_image);
        RCLCPP_INFO(this->get_logger(), "Undistorted image saved as 'undistorted.jpg'.");
    }

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    bool loadCameraInfo(const std::string &file_path) {
        try {
            YAML::Node config = YAML::LoadFile(file_path);

            // 解析 camera_matrix
            auto camera_matrix_data = config["camera_matrix"]["data"].as<std::vector<double>>();
            camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();

            // 解析 distortion_coefficients
            auto dist_coeffs_data = config["distortion_coefficients"]["data"].as<std::vector<double>>();
            dist_coeffs_ = cv::Mat(1, dist_coeffs_data.size(), CV_64F, dist_coeffs_data.data()).clone();

            RCLCPP_INFO(this->get_logger(), "Successfully loaded camera info.");
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing YAML file: %s", e.what());
            return false;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
