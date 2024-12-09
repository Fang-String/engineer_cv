import cv2
import numpy as np
from sensor_msgs_py.point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2, PointField,Image
from cv_bridge import CvBridge
from realsense2_camera_msgs.msg import RGBD
import rclpy
from rclpy.node import Node
import yaml
import struct
import open3d as o3d

def load_camera_info(file_path):
    with open(file_path, 'r') as file:
        camera_info = yaml.safe_load(file)
    return camera_info

def float_to_binary32(value):
    # 将浮点数打包成二进制格式
    packed = struct.pack('!f', value)
    # 将二进制格式转换为整数
    integer = int.from_bytes(packed, byteorder='big')
    # 将整数转换为二进制字符串
    binary_str = format(integer, '032b')
    return binary_str

class RGBDToPointCloud(Node):
    def __init__(self):
        super().__init__('rgbd_to_pointcloud')

        # 订阅 RGBD 图像
        self.rgbd_sub = self.create_subscription(
            RGBD,
            '/camera/camera/rgbd',  # 订阅的 RGBD 图像话题
            self.rgbd_callback,
            10
        )
        self.get_logger().info("Subscribing to RGBD images")

        # 订阅 Mask 图像
        self.mask_sub = self.create_subscription(
            Image,  # 假设 mask 是 Image 类型的消息
            '/camera/camera/mask',  # 订阅的 Mask 图像话题
            self.mask_callback,
            10
        )
        self.get_logger().info("Subscribing to Mask images")

        # 发布点云
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/mypoints', 10)
        self.get_logger().info("Publishing point cloud")

        # CvBridge 实例
        self.bridge = CvBridge()
        self.get_logger().info("CvBridge initialized")

        # 加载相机信息
        camera_info = load_camera_info('config/camera_info.yaml')
        self.camera_matrix = np.array(camera_info['camera_matrix']['data']).reshape(3, 3)
        self.dist_coeffs = np.array(camera_info['distortion_coefficients']['data'])

        # 初始化变量
        self.rgb_image = None
        self.depth_image = None
        self.mask_image = None
        self.points_with_rgb = None

        # 创建一个3秒的定时器
        self.timer = self.create_timer(1.0, self.save_data)

    def mask_callback(self, msg):
        # 转换为 OpenCV 格式
        self.mask_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        self.get_logger().info("Mask image received")

    def rgbd_callback(self, msg):
        # 提取 RGB 图像和深度图像
        rgb_image_msg = msg.rgb  # 假设 RGBD 包含名为 'rgb' 的字段
        depth_image_msg = msg.depth  # 假设 RGBD 包含名为 'depth' 的字段

        # 转换为 OpenCV 格式
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, "32FC1")

        # 打印 frame_id
        self.get_logger().info(f"Frame ID of the RGB image: {rgb_image_msg.header.frame_id}")

        # 检查 mask 图像是否存在
        if self.mask_image is None:
            self.get_logger().warn("No mask image available yet.")
            return

        # 确保 mask 图像大小与深度图像一致
        if self.mask_image.shape != depth_image.shape:
            self.get_logger().warn("Mask image size does not match depth image size.")
            return

        # 相机内参（示例）
        fx, fy, cx, cy = self.camera_matrix[0, 0], self.camera_matrix[1, 1], self.camera_matrix[0, 2], self.camera_matrix[1, 2]

        # 点云计算
        height, width = depth_image.shape
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        points_2d = np.vstack([u.ravel(), v.ravel()]).T

        # 确保 points_2d 是连续的，并且数据类型为 float32
        points_2d = np.ascontiguousarray(points_2d, dtype=np.float32)

        # 去畸变处理
        points_undistorted = cv2.undistortPoints(points_2d.reshape(-1, 1, 2), self.camera_matrix, self.dist_coeffs, P=self.camera_matrix).reshape(-1, 2)
        u_undistorted, v_undistorted = points_undistorted[:, 0], points_undistorted[:, 1]

        # 过滤超出边界的坐标
        mask = (u_undistorted >= 0) & (u_undistorted < width) & (v_undistorted >= 0) & (v_undistorted < height)
        u_undistorted = u_undistorted[mask]
        v_undistorted = v_undistorted[mask]

        # 应用 mask 过滤
        mask_values = self.mask_image[v_undistorted.astype(int), u_undistorted.astype(int)]
        mask_filter = mask_values > 0  # 假设 mask 中白色为 255，黑色为 0

        u_undistorted = u_undistorted[mask_filter]
        v_undistorted = v_undistorted[mask_filter]

        z = depth_image[v_undistorted.astype(int), u_undistorted.astype(int)] / 1000
        x = (u_undistorted - cx) * z / fx
        y = (v_undistorted - cy) * z / fy

        # 过滤无效深度
        valid = (z > 0.35) & (z < 0.8)  # 过滤无效深度
        x, y, z = x[valid], y[valid], z[valid]
        rgb = rgb_image[v_undistorted[valid].astype(int), u_undistorted[valid].astype(int)].astype(np.int32)

        # 打印 rgb 数组的形状进行调试
        self.get_logger().info(f"Shape of rgb after filtering: {rgb.shape}")

        # 检查 rgb 数组的形状
        assert rgb.ndim == 2 and rgb.shape[1] == 3, f"Unexpected shape of rgb: {rgb.shape}"

        rgb_data = (rgb[:, 2] * 256 * 256 + rgb[:, 1] * 256 + rgb[:, 0]).view(dtype=np.float32)

        # 打印 rgb_data 的前几个值进行调试
        self.get_logger().info(f"First few values of rgb_data: {rgb_data[:10]}")

        # 创建点云字段
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # 合并 x, y, z 和 rgb 数据
        points = np.column_stack((x, y, z))  # XYZ 格式

        # 创建结构化的 NumPy 数组
        points_with_rgb = np.zeros(points.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.float32)
        ])
        points_with_rgb['x'] = points[:, 0]
        points_with_rgb['y'] = points[:, 1]
        points_with_rgb['z'] = points[:, 2]
        points_with_rgb['rgb'] = rgb_data

        # 打印 points_with_rgb 的前几个值进行调试
        self.get_logger().info(f"First few values of points_with_rgb: {points_with_rgb[:10]}")

        # 创建点云消息
        header = rgb_image_msg.header
        cloud = create_cloud(header, fields, points_with_rgb)
        cloud.is_dense = True

        # 发布点云
        self.get_logger().info(f"Publishing point cloud with {len(points_with_rgb)} points")
        self.pointcloud_pub.publish(cloud)

        # 保存当前的 RGB 图像和点云数据
        self.rgb_image = rgb_image
        self.points_with_rgb = points_with_rgb

    def save_data(self):
        if self.rgb_image is not None and self.points_with_rgb is not None:
            # 保存 RGB 图像为 PNG 文件
            cv2.imwrite('/home/ubuntu/tools/scripts/img/rgb_image.png', self.rgb_image)
            self.get_logger().info("RGB image saved as rgb_image.png")

            # 提取 x, y, z 列并组合成一个新的 NumPy 数组
            points_xyz = np.column_stack((self.points_with_rgb['x'], self.points_with_rgb['y'], self.points_with_rgb['z']))

            # 提取 rgb 数据并拆分为 r, g, b 通道
            rgb_data = self.points_with_rgb['rgb'].view(np.uint32)
            r = ((rgb_data >> 16) & 0xFF).astype(np.float32) / 255.0
            g = ((rgb_data >> 8) & 0xFF).astype(np.float32) / 255.0
            b = (rgb_data & 0xFF).astype(np.float32) / 255.0

            # 组合成颜色数组
            colors = np.column_stack((r, g, b))

            # 保存点云为 PLY 文件
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_xyz)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            o3d.io.write_point_cloud('/home/ubuntu/tools/scripts/pcd/color_point_cloud.ply', pcd)  # 修改文件扩展名为 .ply
            self.get_logger().info("Point cloud saved as color_point_cloud.ply")

            # 停止定时器
            self.timer.cancel()
        else:
            self.get_logger().warn("No data available to save.")

def main(args=None):
    rclpy.init(args=args)
    node = RGBDToPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

