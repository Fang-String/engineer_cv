#ifndef RGBDP_INFO_HPP
#define RGBDP_INFO_HPP
#define DIST 0.5 //偏移向量阈值
#define NUM 20 //迭代次数阈值


#include "realsense2_camera_msgs/msg/rgbd.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include "opencv4/opencv2/imgproc/imgproc.hpp"
#include <opencv4/opencv2/video/tracking.hpp>
#include "opencv4/opencv2/core/utility.hpp"
#include <opencv4/opencv2/tracking.hpp>
#include <sstream> 
#include <vector>
#include <Eigen/Dense>         // 使用Eigen库处理矩阵
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include "pcl_conversions/pcl_conversions.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/point32.hpp>



const int screenWidth = 1280;
const int screenHeight = 720; 



#endif