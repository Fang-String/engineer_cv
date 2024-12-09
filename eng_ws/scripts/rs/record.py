import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import os

class RGBDSaver(Node):
    def __init__(self):
        super().__init__('rgb_saver')

        # 订阅 RGB 图像
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # 订阅的 RGB 图像话题
            self.rgb_callback,
            10
        )
        self.get_logger().info("Subscribing to RGB images")

        # CvBridge 实例
        self.bridge = CvBridge()

        # 初始化变量
        self.rgb_image = None
        self.image_count = 0

        # 创建一个1秒的定时器
        self.timer = self.create_timer(1.0, self.save_data)

    def rgb_callback(self, msg):
        # 转换为 OpenCV 格式
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.get_logger().info("RGB image received")

    def save_data(self):
        if self.rgb_image is not None:
            # 构建保存路径
            save_path = f'/home/ubuntu/tools/scripts/img2train/rgb_image_{self.image_count + 1}.png'#1可以改成其他数字
            
            # 保存 RGB 图像为 PNG 文件
            cv2.imwrite(save_path, self.rgb_image)
            self.get_logger().info(f"RGB image saved as {os.path.basename(save_path)}")

            # 更新计数器
            self.image_count += 1
        else:
            self.get_logger().warn("No RGB image available to save.")

def main(args=None):
    rclpy.init(args=args)
    node = RGBDSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()