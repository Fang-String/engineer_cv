import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge
import cv2
import os

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(SensorImage, 'tmpmask', 10)
        self.bridge = CvBridge()
        self.timer_period = 1.0  # 发布频率为1Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.image_path = "/home/ubuntu/tools/engineer_cv/eng_ws/scripts/rs/img/maskrgb_imagea.png"  # 替换为你的图片路径
        self.get_logger().info(f"Loading image from {self.image_path}")

    def timer_callback(self):
        if not os.path.exists(self.image_path):
            self.get_logger().error(f"Image file {self.image_path} does not exist.")
            return

        try:
            cv_image = cv2.imread(self.image_path)
            if cv_image is None:
                self.get_logger().error(f"Failed to load image from {self.image_path}.")
                return

            sensor_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.publisher_.publish(sensor_image_msg)
            self.get_logger().info('Publishing image')
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()