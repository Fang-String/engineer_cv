#ifndef CONFIG_HPP
#include <iostream>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#define CONFIG_HPP
struct Config {
    std::string rgbdTopic;
    std::string src_cloud;
    std::string tar_cloud;
    bool downsample;
    bool estimate_normal;
    double voxel_resolution;
    double normal_est_radius;
    double search_radius;
    int icp_method;
    double icp_max_corres_dist;
    double icp_transformation_epsilon;
    int icp_max_iterations;
    double color_icp_lambda;
    int color_weight;
    bool color_visualization;
    bool precloud;


    explicit Config(std::shared_ptr<rclcpp::Node> nh) {
        rgbdTopic = nh->declare_parameter<std::string>("RgbdTopic", "");
        src_cloud = nh->declare_parameter<std::string>("Src_Cloud", "");
        tar_cloud = nh->declare_parameter<std::string>("Tar_Cloud", "");
        downsample=nh->declare_parameter<bool>("Downsample",true); 
        estimate_normal=nh->declare_parameter<bool>("Estimate_Normal", true);
        voxel_resolution=nh->declare_parameter<double>("Voxel_Resolusion", 0.01);
        normal_est_radius=nh->declare_parameter<double>("Normal_Est_Radius", 0.01);
        search_radius=nh->declare_parameter<double>("Search_Radius", 0);
        icp_method=nh->declare_parameter<int>("Icp_Method", 0);
        icp_max_corres_dist=nh->declare_parameter<double>("Icp_Max_Corres_Dist", 0);
        icp_transformation_epsilon=nh->declare_parameter<double>("Icp_Transformation_Epsilon", 0);
        icp_max_iterations=nh->declare_parameter<int>("Icp_Max_Iterations", 0);
        color_icp_lambda=nh->declare_parameter<double>("Color_Icp_Lambda", 0);
        color_weight=nh->declare_parameter<int>("Color_Weight", 0);
        color_visualization=nh->declare_parameter<bool>("Color_Visualization", true);
        precloud=nh->declare_parameter<bool>("Precloud", true);

        
    }
};



#endif