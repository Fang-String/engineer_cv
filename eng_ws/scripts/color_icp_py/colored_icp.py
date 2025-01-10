#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from std_msgs.msg import Int8,Int32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
import copy
import time
import transforms3d.quaternions as tf_quaternions

def draw_registration_result(source, target, transformation, color=False):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    if not color:
        source_temp.paint_uniform_color([1, 1, 0])
        target_temp.paint_uniform_color([0, 1, 1])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def decode_rgb_from_float32(float32_array):
    """
    将一个(n, 1)的float32列表转换为RGB值的(n, 3)列表
    :param float32_array: 输入的(n, 1) float32列表，包含编码为00RRGGBB的RGB数据
    :return: (n, 3)的RGB列表
    """
    # 创建一个空的RGB列表
    rgb_list = []
    
    # 遍历每个 float32 数字
    for val in float32_array:
        # 将 float32 数值转换为字节（32位）
        byte_data = np.frombuffer(np.float32(val).tobytes(), dtype=np.uint8)
        
        # 提取后三个字节作为RGB值
        r = byte_data[1]  # 红色通道，第二个字节
        g = byte_data[2]  # 绿色通道，第三个字节
        b = byte_data[3]  # 蓝色通道，第四个字节
        
        # 将 R, G, B 组成元组并添加到 rgb_list
        rgb_list.append([r, g, b])
    
    # 转换为 (n, 3) 的 numpy 数组
    rgb_list = np.array(rgb_list)/255
    return rgb_list
# 提取 ROS 点云消息并转换为 Open3D 点云
def convert_ros_point_cloud_to_open3d(pc_msg):
    # 使用 ROS 的 point_cloud2 来获取点云中的数据
    pc_data = pc2.read_points(pc_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
    print(pc_data[0][0])
    
    # 将生成器转换为列表
    color_tmp = [point[3] for point in pc_data]
    x = [point[0] for point in pc_data]
    y = [point[1] for point in pc_data]
    z = [point[2] for point in pc_data]
    points = np.array(list(zip(x, y, z)))
    colors = decode_rgb_from_float32(color_tmp)

    # 创建 Open3D 点云
    o3d_pc = o3d.geometry.PointCloud()
    o3d_pc.points = o3d.utility.Vector3dVector(points)
    o3d_pc.colors = o3d.utility.Vector3dVector(colors)

    return o3d_pc

class PointCloudRegistrationNode(Node):
    def __init__(self):
        super().__init__('point_cloud_registration_node')

        # 初始化源点云选择标志
        self.src_choice = 1  # 默认为选择第一个源点云

        # 订阅选择源点云的int消息
        self.create_subscription(Int8, '/mine_color', self.src_choice_callback, 10)

        # 订阅初始变换pose消息
        self.create_subscription(Pose, '/control_tf', self.pose_callback, 10)

        # 订阅源点云消息 (PointCloud2)
        self.create_subscription(PointCloud2, '/mine_cloud', self.point_cloud_callback, 10)
        
        self.subscription = self.create_subscription(Float32MultiArray,'/mine_mat',self.listener_callback,10)

        # 发布最终的配准结果Pose消息
        self.pose_pub = self.create_publisher(Pose, 'mine_tf', 10)

        # 初始化目标点云
        self.mine_color = 1
        self.src_path = "/home/ubuntu/step2pcd/cube" + str(self.mine_color) + ".pcd"
        self.source_cloud = o3d.io.read_point_cloud(self.src_path)  # 目标点云
        self.target_cloud = None  # 初始为空
        self.initial_transform = None
        
        

        self.get_logger().info("Node initialized.")
        

    def src_choice_callback(self, msg: Int32):
        # 更新选择的源点云
        self.mine_color = msg.data
        self.get_logger().info(f"Source point cloud choice updated to {self.mine_color}")

    def pose_callback(self, msg: Pose):
        # 处理接收到的Pose消息，将其转换为变换矩阵并应用
        self.get_logger().info(f"Received initial transform pose: {msg}")

        # 将接收到的Pose转换为变换矩阵
        transformation = self.pose_to_transformation_matrix(msg)
        self.initial_transform = transformation
        print(self.initial_transform)

        
        # # 执行配准
        # if self.source_cloud is not None:
        #     self.perform_registration()
        # else:
        #     self.get_logger().warning("Source point cloud not received yet.")

    def listener_callback(self, msg):
        # 从消息中获取展平的矩阵数据
        flat_matrix = np.array(msg.data)

        # 将展平的矩阵重新 reshape 成 4x4
        transformation_matrix = flat_matrix.reshape(4, 4)

        self.initial_transform = transformation_matrix

        # 打印接收到的 4x4 变换矩阵
        self.get_logger().info(f'Received transform matrix: {transformation_matrix}')
    def point_cloud_callback(self, msg: PointCloud2):
        # 处理接收到的点云消息，将其转换为Open3D点云
        self.get_logger().info(f"Received new target point cloud.")

        # 将PointCloud2消息转换为Open3D点云
        o3d_pc = convert_ros_point_cloud_to_open3d(msg)


        self.target_cloud = o3d.geometry.PointCloud()
        self.target_cloud.points = o3d.utility.Vector3dVector(o3d_pc.points)
        self.target_cloud.colors = o3d.utility.Vector3dVector(o3d_pc.colors)

        # 如果收到了初始变换，则执行配准
        if self.initial_transform is not None:
            self.perform_registration()
    def pose_to_transformation_matrix(self, pose: Pose):
        # 将Pose消息转换为4x4变换矩阵
        rotation_matrix = tf_quaternions.quat2mat([pose.orientation.w,pose.orientation.x, pose.orientation.y, pose.orientation.z])
        transformation = np.identity(4)
        transformation[:3, :3] = rotation_matrix
        transformation[0, 3] = pose.position.x
        transformation[1, 3] = pose.position.y
        transformation[2, 3] = pose.position.z
        return transformation

    def colored_icp_registration(self, source, target, voxel_size, init_trans=np.identity(4)):
        
        self.get_logger().info(f"Starting colored ICP registration,received points:{len(target.points)}")
        # draw_registration_result(source, target, init_trans, color=True)
        
        # 配准过程 (你可以调整这里的具体细节)
        voxel_radius = [5 * voxel_size, 3 * voxel_size, voxel_size]
        max_iter = [40, 40, 40]
        current_transformation = np.identity(4)
        source_cp = copy.deepcopy(source)
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
                    relative_fitness=1e-7,
                    relative_rmse=1e-7,
                    max_iteration=max_it)
            )

            current_transformation = result.transformation
        current_transformation = current_transformation@init_trans
        return current_transformation

    def perform_registration(self):
        
        # 根据选择的源点云和收到的目标点云和粗tf执行配准
        if self.target_cloud is None:
            self.get_logger().warning("Source point cloud is not available yet.")
            return
        start_time = time.time()
        # 假设目标点云已经加载
        voxel_size = 0.01
        final_transformation = self.colored_icp_registration(self.source_cloud, self.target_cloud, voxel_size, self.initial_transform)
        

        # 将变换矩阵转换为Pose消息
        pose_msg = self.transform_to_pose(final_transformation)

        # 发布最终的配准结果
        self.pose_pub.publish(pose_msg)
        print(final_transformation)
        # draw_registration_result(self.source_cloud, self.target_cloud, final_transformation, color=True)
        end_time = time.time()
        self.get_logger().info("Final transformation pose published,time used: %f"%(end_time - start_time))

    def transform_to_pose(self, transformation: np.ndarray) -> Pose:
        # 将变换矩阵转换为Pose消息
        pose_msg = Pose()

        # 提取旋转部分并转换为四元数
        rotation_matrix = transformation[:3, :3]
        quat = tf_quaternions.mat2quat(rotation_matrix)
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]

        # 提取位移部分
        pose_msg.position.x = transformation[0, 3]
        pose_msg.position.y = transformation[1, 3]
        pose_msg.position.z = transformation[2, 3]

        return pose_msg

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
