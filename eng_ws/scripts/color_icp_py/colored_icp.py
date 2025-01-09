#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from std_msgs.msg import Int32
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudRegistrationNode(Node):
    def __init__(self):
        super().__init__('point_cloud_registration_node')

        # 初始化源点云选择标志
        self.src_choice = 1  # 默认为选择第一个源点云

        # 订阅选择源点云的int消息
        self.create_subscription(Int32, '/mine_color', self.src_choice_callback, 10)

        # 订阅初始变换pose消息
        self.create_subscription(Pose, '/control_tf', self.pose_callback, 10)

        # 订阅源点云消息 (PointCloud2)
        self.create_subscription(PointCloud2, '/mine_cloud', self.point_cloud_callback, 10)

        # 发布最终的配准结果Pose消息
        self.pose_pub = self.create_publisher(Pose, 'mine_tf', 10)

        # 初始化目标点云
        self.mine_color = 1
        self.tar_path = "/path/to/target" + str(self.mine_color) + ".pcd"
        self.target_cloud = o3d.io.read_point_cloud(self.tar_path)  # 目标点云
        self.source_cloud = None  # 初始为空
        self.initial_transform = None
        
        

        self.get_logger().info("Node initialized.")
        
        # 初始变换 (在收到消息之前设定为空)
        self.initial_transform = np.identity(4)

    def src_choice_callback(self, msg: Int32):
        # 更新选择的源点云
        self.mine_color = msg.data
        self.get_logger().info(f"Source point cloud choice updated to {self.mine_color}")

    def pose_callback(self, msg: Pose):
        # 处理接收到的Pose消息，将其转换为变换矩阵并应用
        self.get_logger().info(f"Received initial transform pose: {msg}")

        # 将接收到的Pose转换为变换矩阵
        transformation = self.pose_to_transformation_matrix(msg)
        
        # 执行配准
        if self.source_cloud is not None:
            self.perform_registration(transformation)
        else:
            self.get_logger().warning("Source point cloud not received yet.")

    def point_cloud_callback(self, msg: PointCloud2):
        # 处理接收到的点云消息，将其转换为Open3D点云
        self.get_logger().info(f"Received new source point cloud.")

        # 将PointCloud2消息转换为Open3D点云
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(pc_data))
        self.source_cloud = o3d.geometry.PointCloud()
        self.source_cloud.points = o3d.utility.Vector3dVector(points)

        # 如果收到了初始变换，则执行配准
        if self.initial_transform is not None:
            self.perform_registration(self.initial_transform)

    def pose_to_transformation_matrix(self, pose: Pose):
        # 将Pose消息转换为4x4变换矩阵
        rotation_matrix = self.quaternion_to_rotation_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        transformation = np.identity(4)
        transformation[:3, :3] = rotation_matrix
        transformation[0, 3] = pose.position.x
        transformation[1, 3] = pose.position.y
        transformation[2, 3] = pose.position.z
        return transformation

    def quaternion_to_rotation_matrix(self, quat):
        # 将四元数转换为旋转矩阵
        q0, q1, q2, q3 = quat
        R = np.array([
            [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1**2 + q2**2)]
        ])
        return R

    def colored_icp_registration(self, source, target, voxel_size, init_trans=np.identity(4)):
        self.get_logger().info("Starting colored ICP registration")
        
        # 配准过程 (你可以调整这里的具体细节)
        voxel_radius = [5 * voxel_size, 3 * voxel_size, voxel_size]
        max_iter = [70, 45, 30]
        current_transformation = np.identity(4)
        source_cp = source.deepcopy()
        source_cp.transform(init_trans)

        for scale in range(3):
            radius = voxel_radius[scale]
            max_it = max_iter[scale]
            self.get_logger().info(f"scale_level = {scale}, voxel_size = {radius}, max_iter = {max_it}")

            source_down = source_cp.voxel_down_sample(radius)
            target_down = target.voxel_down_sample(radius)

            source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=20))
            target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=20))

            result = o3d.pipelines.registration.registration_colored_icp(
                source_down, 
                target_down, 
                radius, 
                current_transformation,
                o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-6,
                    relative_rmse=1e-6,
                    max_iteration=max_it)
            )

            current_transformation = result.transformation

        return current_transformation

    def perform_registration(self):
        # 根据选择的源点云执行配准
        if self.source_cloud is None:
            self.get_logger().warning("Source point cloud is not available yet.")
            return

        # 假设目标点云已经加载
        voxel_size = 0.01
        final_transformation = self.colored_icp_registration(self.source_cloud, self.target_cloud, voxel_size, self.init_trans)

        # 将变换矩阵转换为Pose消息
        pose_msg = self.transform_to_pose(final_transformation)

        # 发布最终的配准结果
        self.pose_pub.publish(pose_msg)
        self.get_logger().info("Final transformation pose published.")

        # 将最终的变换应用到源点云并发布
        self.source_cloud.transform(final_transformation)

        # 转换为PointCloud2并发布
        src_cloud_msg = self.convert_o3d_to_ros_point_cloud(self.source_cloud)
        self.publisher.publish(src_cloud_msg)
        self.get_logger().info("Aligned source point cloud published.")

    def transform_to_pose(self, transformation: np.ndarray) -> Pose:
        # 将变换矩阵转换为Pose消息
        pose_msg = Pose()

        # 提取旋转部分并转换为四元数
        rotation_matrix = transformation[:3, :3]
        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]

        # 提取位移部分
        pose_msg.position.x = transformation[0, 3]
        pose_msg.position.y = transformation[1, 3]
        pose_msg.position.z = transformation[2, 3]

        return pose_msg

    def rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray):
        # 将旋转矩阵转换为四元数
        q0 = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
        q1 = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * q0)
        q2 = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * q0)
        q3 = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * q0)
        return [q1, q2, q3, q0]

    def convert_o3d_to_ros_point_cloud(self, o3d_cloud):
        # 将Open3D点云转换为PointCloud2
        points = np.asarray(o3d_cloud.points)
        header = std_msgs.msg.Header()
        header.stamp = rclpy.time.Time().to_msg()
        header.frame_id = ""  # 这里使用你需要的frame_id

        # 创建 PointCloud2 消息
        pc_data = pc2.create_cloud_xyz32(header, points)
        return pc_data

def main(args=None):
    rclpy.init(args=args)

    # 创建节点实例
    node = PointCloudRegistrationNode()

    try:
        # 在此运行任何需要的操作
        while rclpy.ok():
            # 执行配准
            node.perform_registration()
            rclpy.spin_once(node)  # 每次运行循环时调用一次 spin_once
    except KeyboardInterrupt:
        pass  # 处理 Ctrl+C 中断

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
