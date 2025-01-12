#include "rgbdp_info/rgbdp_info.hpp"
#include <pcl/impl/point_types.hpp>




class RGBDPointInfoNode : public rclcpp::Node
{
public:
  RGBDPointInfoNode()
  : Node("rgbd_point_info_node")
  {
    initial = true;
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
    rgbd_sub_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
      "/camera/camera/rgbd", 10, std::bind(&RGBDPointInfoNode::rgbd_callback, this, std::placeholders::_1));
    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/mymask", 10, std::bind(&RGBDPointInfoNode::mask_callback, this, std::placeholders::_1));
    cv::namedWindow("RGB Image", cv::WINDOW_NORMAL);
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mypointscpp",10);
  }


private:
  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher_;
  bool initial;
  int status;
  cv::Mat rgb_image;
  cv::Mat depth_image;
  cv::Mat mask;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg)
  {
      // 将ROS图像消息转换为OpenCV图像
    rgb_image = cv_bridge::toCvCopy(msg->rgb, "bgr8")->image;
    depth_image = cv_bridge::toCvCopy(msg->depth, "32FC1")->image;
    rgbd_pcl(rgb_image,depth_image);
  }

  void mask_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
      // 将ROS图像消息转换为OpenCV图像
    mask = cv_bridge::toCvCopy(msg, "mono8")->image;

  }

  void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {

      
    }
  }
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

int rgbd_pcl(cv::Mat rgbImage,cv::Mat depthImage) {
    // 假设depth_image和rgbImage是您的深度图和RGB
    cv::Mat rgb_image_ = rgbImage;
    cv::Mat depth_image_ = depthImage;
    cv::Mat rgb_image_u = rgb_image_.clone();


    if (depth_image.empty() ) {
        std::cerr << "无法读取图像!" << std::endl;
        return -1;
    }

    int height = depth_image.rows;
    int width = depth_image.cols;

    // 相机内参
    double fx = camera_matrix_.at<double>(0, 0);
    double fy = camera_matrix_.at<double>(1, 1);
    double cx = camera_matrix_.at<double>(0, 2);
    double cy = camera_matrix_.at<double>(1, 2);
    //去畸变
    cv::undistort(rgb_image_u,rgb_image,camera_matrix_,dist_coeffs_);


    // 存储点云数据
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr mycloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int v = 0; v < height; ++v) {
          for (int u = 0; u < width; ++u) {
            float z;
            // if(mask.at<int>(v,u)>0)
            {z = depth_image.at<float>(v, u)/1000;}// 读取深度
            // else {z=0;}
            if (z > 0.35&&z<0.8) { // 确保深度值有效
                double x = (u - cx) * z / fx;
                double y = (v - cy) * z / fy;
                pcl::PointXYZRGB point;
                cv::Vec3b color = rgb_image.at<cv::Vec3b>(v, u);
                uint8_t r = color[2];  // OpenCV 的颜色顺序为 BGR
                uint8_t g = color[1];
                uint8_t b = color[0];
                point.x = x;
                point.y = y;
                point.z = z;//相机坐标系和世界坐标系转换
                point.r = r;
                point.g = g;
                point.b = b;
                mycloud->push_back(point);// 存储三维坐标
                
            }
        }
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*mycloud,cloud_msg);
    cloud_msg.header.frame_id = "camera_color_optical_frame";
    cloud_msg.header.stamp = this->now();
    pc_publisher_->publish(cloud_msg);
    RCLCPP_INFO(this->get_logger(), "pc pubed %d",int(mycloud->size()));

    return 0;
}

int run(cv::Mat &src,cv::Mat &canvas,cv::Mat &dep) {
    // 读取图像
    cv::Mat image = src.clone();
    if (image.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
        return -1;
    }

    // 将图像转换到 HSV 色彩空间
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

    // 定义黄色的 HSV 范围
    cv::Scalar lowerYellow(14, 120, 130);
    cv::Scalar upperYellow(34, 215, 225);

    // 创建掩码
    cv::Mat mask;
    cv::inRange(hsvImage, lowerYellow, upperYellow, mask);

    // 进行形态学操作以去除噪声
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    // 找到轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 在原图上绘制轮廓
    cv::Mat output = image.clone();
    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(output, contours, (int)i, cv::Scalar(0, 255, 0), 2);
    }

    // 显示结果
    //cv::imshow("Original Image", image);
    cv::imshow("Mask", mask);
    canvas = mask.clone();
    cv::imshow("Contours", output);
    //cv::imwrite("output.jpg", output);
    cv::waitKey(40);

    return 0;
}

int rectshape(cv::Mat &src,cv::Mat &mask,cv::Mat depth_i) {
    // 读取图像
    
    cv::Mat image = src.clone();

    if (image.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
        return -1;
    }

    // 找到轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); i++) {
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

        // 获取旋转矩形的四个顶点
        cv::Point2f vertices[4];
        rotatedRect.points(vertices);

        // 输出四个顶点坐标
        std::cout << "Rotated Rectangle Corners:" << std::endl;
        for (int j = 0; j < 4; j++) {
            std::cout << "Vertex " << j + 1 << ": (" << vertices[j].x << ", " << vertices[j].y << ")" << std::endl;

            // 在图像上绘制顶点
            cv::circle(image, vertices[j], 5, cv::Scalar(255, 0, 0), -1); // 绘制顶点
            cv::putText(image, "(" + std::to_string(depth_i.at<float>(vertices[j].y,vertices[j].x))+")", 
                        vertices[j] + cv::Point2f(10, -10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1); // 标注坐标
        } }



    // 显示结果图像
    cv::imshow("Image with Rotated Rectangles", image);
    cv::waitKey(40); // 等待按键

      return 0;
}

int getHSV() {
    // 读取图像
    cv::Mat image = cv::imread("output.jpg");
    if (image.empty()) {
        std::cerr << "Could not open or find the image!" << std::endl;
        return -1;
    }

    // 将图像转换到 HSV 色彩空间
    cv::Mat hsvImage;
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

    // 定义感兴趣区域（ROI），这里以 (x, y, width, height) 定义
    int x = 600; // ROI 的左上角 x 坐标
    int y = 300; // ROI 的左上角 y 坐标
    int width = 100; // ROI 的宽度
    int height = 100; // ROI 的高度
    cv::rectangle(image, cv::Point(x, y), cv::Point(x + width, y + height), cv::Scalar(0, 255, 0), 2); // 绿色边框，线宽为 2

    // 提取 ROI
    cv::Rect roi(x, y, width, height);
    cv::Mat roiImage = hsvImage(roi);

    // 计算平均 HSV 值
    cv::Scalar avgHSV = cv::mean(roiImage);

    std::cout << "Average HSV Value in the Region:" << std::endl;
    std::cout << "H: " << avgHSV[0] << std::endl; // Hue
    std::cout << "S: " << avgHSV[1] << std::endl; // Saturation
    std::cout << "V: " << avgHSV[2] << std::endl; // Value
    cv::imshow("Contours", image);
    cv::waitKey(10);

    return 0;
}
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGBDPointInfoNode>());
  rclcpp::shutdown();
  return 0;
}



