import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        # 创建发布者对象，发布到 "image_topic" 话题
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Image, '/mymask', qos_profile)

        # 设置定时器，每秒发布一次图像
        self.timer = self.create_timer(1.0, self.timer_callback)

        # CvBridge 实例，帮助转换 OpenCV 图像到 ROS 图像
        self.bridge = CvBridge()

        # 载入一张示例图像
        self.image = cv2.imread('/home/ubuntu/tools/engineer_cv/eng_ws/scripts/rs/img/maskrgb_imagea.png')

        if self.image is None:
            self.get_logger().error('Failed to load image.')
        else:
            self.get_logger().info('Image loaded successfully.')

    def timer_callback(self):
        # 将 OpenCV 图像转换为 ROS 图像消息
        if self.image is not None:
            try:
                # 转换为灰度图像并发布
                ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
                self.publisher_.publish(ros_image)
                self.get_logger().info('Publishing image...')
            except Exception as e:
                self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
