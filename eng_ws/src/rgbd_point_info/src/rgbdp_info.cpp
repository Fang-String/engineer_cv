#include "rgbdp_info/rgbdp_info.hpp"
#include <pcl/impl/point_types.hpp>




class RGBDPointInfoNode : public rclcpp::Node
{
public:
  RGBDPointInfoNode()
  : Node("rgbd_point_info_node")
  {
    initial = true;
    rgbd_sub_ = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
      "/camera/camera/rgbd", 10, std::bind(&RGBDPointInfoNode::rgbd_callback, this, std::placeholders::_1));
    cv::namedWindow("RGB Image", cv::WINDOW_NORMAL);
    pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/depth_cloud",10);
  }


private:
  void rgbd_callback(const realsense2_camera_msgs::msg::RGBD::SharedPtr msg)
  {
    try
    {
      // 将ROS图像消息转换为OpenCV图像
      cv::Mat rgb_image = cv_bridge::toCvCopy(msg->rgb, "bgr8")->image;
      cv::Mat depth_image = cv_bridge::toCvCopy(msg->depth, "32FC1")->image;
      //std::cerr << rgb_image.rows<<","<<rgb_image.cols<< ","<<depth_image.rows<<","<<depth_image.cols<< std::endl;
      run(rgb_image, canvas,depth_image);
      //getHSV();
      rectshape(rgb_image,canvas, depth_image);
      rgbd_pcl(depth_image );
      
      }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception occurred");
    }
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




int rgbd_pcl(cv::Mat &depthImage) {
    // 假设depthImage和rgbImage是您的深度图和RGB

    if (depthImage.empty() ) {
        std::cerr << "无法读取图像!" << std::endl;
        return -1;
    }

    int height = depthImage.rows;
    int width = depthImage.cols;

    // 相机内参
    double fx = 431.1919860839844*1280/848;
    double fy = 431.1919860839844*720/480;
    double cx = 430.00750732421875*1280/848;
    double cy = 238.7722930908203*720/480;

    // 存储点云数据
        pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int v = 0; v < height; ++v) {
          for (int u = 0; u < width; ++u) {
            float z = depthImage.at<float>(v, u)/1000; // 读取深度值
            if (z > 0.2) { // 确保深度值有效
                double x = (u - cx) * z / fx;
                double y = (v - cy) * z / fy;
                pcl::PointXYZ point;
                Eigen::Vector3d ptmp(z,-x,-y);
                // Eigen::Matrix3d R;
                // R<<0,0,1,-1,0,0,0,-1,0;
                // ptmp(0) = x;
                // ptmp(1) = y;
                // ptmp(2) = z;
                // ptmp = R*ptmp;
                point.x = ptmp(0);
                point.y = ptmp(1);
                point.z = ptmp(2);


                depth_cloud->push_back(point);// 存储三维坐标
                
            }
        }
    }
    sensor_msgs::msg::PointCloud2 cloud_msg;
    //pcl::toROSMsg(*grid_cloud, cloud_msg);
    pcl::toROSMsg(*depth_cloud,cloud_msg);
    cloud_msg.header.frame_id = "base_link";
    cloud_msg.header.stamp = this->now();
    pc_publisher_->publish(cloud_msg);

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


  rclcpp::Subscription<realsense2_camera_msgs::msg::RGBD>::SharedPtr rgbd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;

  cv::Rect targetMineral; //目标
  cv::Point center;//目标中心
  cv::Rect2d lastRect = targetMineral;
  bool initial;
  bool success;
  std::vector<cv::Rect> detectionBoxes;//检测框
  cv::Mat rgb_image;
  cv::Mat depth_image;
  cv::Mat canvas;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RGBDPointInfoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



