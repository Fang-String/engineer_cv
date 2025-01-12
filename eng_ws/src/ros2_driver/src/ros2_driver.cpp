#include <rclcpp/rclcpp.hpp>
#include <cstdlib>  // For std::system
#include <chrono>   // For std::chrono::seconds
#include <thread>  
// class MyRosNode : public rclcpp::Node
// {
// public:
//     MyRosNode() : Node("my_ros2_node")
//     {
//         // 打印日志信息
//         RCLCPP_INFO(this->get_logger(), "ROS 2 Node started");

//         // 启动 Python 脚本
//         start_python_script();
//     }

// private:
    void start_python_script()
    {
        // 使用 system() 启动 Python 脚本
        int result = std::system("/home/ubuntu/tools/engineer_cv/eng_ws/scripts/sam_4_eng/samuel/bin/python /home/ubuntu/tools/engineer_cv/eng_ws/scripts/sam_4_eng/sam.py");

        // // 检查脚本是否成功启动
        // if (result == 0)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Python script started successfully.");
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to start Python script.");
        // }
    }
// };

int main(int argc, char ** argv)
{
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<MyRosNode>());
    // rclcpp::shutdown();
    // return 0;
    start_python_script();
    // std::this_thread::sleep_for(std::chrono::seconds(5));

    return 0;

}
