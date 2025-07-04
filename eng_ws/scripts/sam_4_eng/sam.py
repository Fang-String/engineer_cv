import os
import numpy as np
import torch
import matplotlib.pyplot as plt
import cv2
import rclpy
from PIL import Image as PILImage  # 使用别名避免命名冲突
from segment_anything import sam_model_registry, SamPredictor
from sensor_msgs_py.point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2, PointField, Image as SensorImage  # 使用别名避免命名冲突
from cv_bridge import CvBridge
from realsense2_camera_msgs.msg import RGBD
from rclpy.node import Node
from std_msgs.msg import Bool
import yaml
import struct
import open3d as o3d

sam_checkpoint = "/home/ubuntu/tools/engineer_cv/eng_ws/scripts/sam_4_eng/data/sam_vit_b_01ec64.pth"
model_type = "vit_b"
device = "cuda"

sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
sam.to(device=device)



# 设置输入文件夹和输出文件夹
input_folder = "/home/ubuntu/tools/engineer_cv/eng_ws/scripts/sam_4_eng/images"  # 输入图片文件夹
output_folder = "/home/ubuntu/tools/engineer_cv/eng_ws/scripts/sam_4_eng/masks"  # 输出掩码文件夹

# 控制标记目标的状态
current_target = 1  # 默认标记第一组目标
coords_list1, labels_list1 = [], []  # 目标 1 的标记点和标签
# coords_list2, labels_list2 = [], []  # 目标 2 的标记点和标签

def show_mask(mask, ax, random_color=False):
    """显示掩码"""
    if random_color:
        color = np.concatenate([np.random.random(3), np.array([0.6])], axis=0)
    else:
        color = np.array([30/255, 144/255, 255/255, 0.6])  # 默认颜色
    h, w = mask.shape[-2:]
    mask_image = mask.reshape(h, w, 1) * color.reshape(1, 1, -1)
    ax.imshow(mask_image)

def show_points(coords, labels, ax, marker_size=375, target=1):
    """显示目标的标记点，目标 1 和目标 2 用不同颜色区分"""
    # 根据目标区分颜色
    if target == 1:
        pos_color = 'red'  # 目标 1 用红色
        neg_color = 'red'  # 目标 1 的负样本用红色叉
    # else:
    #     pos_color = 'green'  # 目标 2 用绿色
    #     neg_color = 'green'  # 目标 2 的负样本用绿色叉

    # 正样本（星形标记）
    pos_points = coords[labels == 1]
    ax.scatter(pos_points[:, 0], pos_points[:, 1], color=pos_color, marker='*', s=marker_size, facecolor=pos_color, linewidth=1.25)

    # 负样本（叉形标记）
    neg_points = coords[labels == 0]
    ax.scatter(neg_points[:, 0], neg_points[:, 1], color=neg_color, marker='x', s=marker_size, facecolor=neg_color, linewidth=1.25)

def on_click(event, coords_list, labels_list, ax):
    """响应鼠标点击事件，标记目标点"""
    if event.button == 1:  # 左键点击
        label = 1
    elif event.button == 3:  # 右键点击
        label = 0
    else:
        return

    if event.inaxes is not None:
        coords_list.append((event.xdata, event.ydata))
        labels_list.append(label)
        print(f"Clicked at ({event.xdata}, {event.ydata}) with button {event.button} (label: {label})")

        # 在图像上显示标记
        color = 'red' if current_target == 1 else 'green'
        marker = '*' if label == 1 else 'x'
        ax.scatter(event.xdata, event.ydata, color=color, marker=marker, s=375, facecolor=color, linewidth=1.25)
        plt.draw()

def on_key(event):
    """响应键盘事件，切换目标"""
    global current_target
    # if event.key == 'a':  # 按下 'A' 键切换目标
        # if current_target == 1:
        #     current_target = 2
        #     print("Switched to Target 2")
        # else:
        #     current_target = 1
        #     print("Switched to Target 1")
        # plt.draw()  # 更新显示

def process_image(image_, predictor, choice=0):
    # 读取图像
    if choice == 0:
        image = cv2.imread(image_)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    else:
        image = image_.copy()
    # 设置图像到 predictor
    predictor.set_image(image)
    # 设置图形的尺寸为 1920x1440
    dpi = 100  # 假设屏幕的 DPI
    fig_width = 1920 / dpi  # 图形的宽度（英寸）
    fig_height = 1440 / dpi  # 图形的高度（英寸）
    # 创建图形对象，设置大小和 DPI
    fig, ax = plt.subplots(figsize=(fig_width, fig_height), dpi=dpi)

    # 连接鼠标点击事件和键盘事件
    fig.canvas.mpl_connect('button_press_event', lambda event: on_click(event, 
                                                                        coords_list1 if current_target == 1 else coords_list2, 
                                                                        labels_list1 if current_target == 1 else labels_list2, 
                                                                        ax))
    fig.canvas.mpl_connect('key_press_event', on_key)  # 绑定按键事件
    
    # 显示图像供用户标注点
    ax.set_title("Click on the image to record points and labels, then press 'A' to switch target.")
    ax.imshow(image)
    plt.show()

    # 获取用户标注的点和标签
    input_point1 = np.array(coords_list1)
    input_label1 = np.array(labels_list1)
    # input_point2 = np.array(coords_list2)
    # input_label2 = np.array(labels_list2)

    # 初次预测（目标1）
    masks, scores, logits = predictor.predict(
        point_coords=input_point1,
        point_labels=input_label1,
        multimask_output=True,
    )
    mask_input = logits[np.argmax(scores), :, :]

    # 精细化预测（目标1）
    masks, _, _ = predictor.predict(
        point_coords=input_point1,
        point_labels=input_label1,
        mask_input=mask_input[None, :, :],
        multimask_output=False,
    )
    maska = masks[0]  # 获取第一个掩码

    # # 生成第二个目标的掩码（目标2）
    # masks, scores, logits = predictor.predict(
    #     point_coords=input_point2,
    #     point_labels=input_label2,
    #     multimask_output=True,
    # )
    # mask_input = logits[np.argmax(scores), :, :]

    # # 精细化预测（目标2）
    # masks, _, _ = predictor.predict(
    #     point_coords=input_point2,
    #     point_labels=input_label2,
    #     mask_input=mask_input[None, :, :],
    #     multimask_output=False,
    # )
    # maskb = masks[0]

    # 高亮原图
    overlay = image.copy()
    overlay[maska > 0] = [255, 0, 0]  # 红色高亮目标 1
    # overlay[maskb > 0] = [0, 0, 255]  # 蓝色高亮目标 2

    # 显示高亮原图 + 掩码
    dpi = 100  # 假设屏幕的 DPI（每英寸像素数）
    fig_width = 1920 / dpi  # 图形的宽度（英寸）
    fig_height = 1440 / dpi  # 图形的高度（英寸）
    fig, axes = plt.subplots(1, 3, figsize=(fig_width, fig_height), dpi=dpi)

    # 显示高亮原图
    axes[0].imshow(overlay)
    axes[0].set_title("Original Image with Highlighted Targets")
    axes[0].axis("off")
    show_points(input_point1, input_label1, axes[0], target=1)  # 显示目标 1 标记点
    # show_points(input_point2, input_label2, axes[0], target=2)  # 显示目标 2 标记点

    # 显示目标 1 掩码
    axes[1].imshow(maska, cmap="gray")
    axes[1].set_title("Target 1 Mask")
    axes[1].axis("off")

    # # 显示目标 2 掩码
    # axes[2].imshow(maskb, cmap="gray")
    # axes[2].set_title("Target 2 Mask")
    # axes[2].axis("off")

    plt.tight_layout()

    # 设置为全屏显示
    # 设置图形的尺寸为 1920x1080

    plt.show()
    coords_list1.clear()
    labels_list1.clear()

    # 将掩码保存为 PIL 图像并转换为 numpy 数组
    mask_image1 = PILImage.fromarray((maska * 255).astype(np.uint8))
    mask_image1_np = np.array(mask_image1)
    # mask_image2 = Image.fromarray((maskb * 255).astype(np.uint8))
    return mask_image1_np  #, mask_image2

def batch_process_images(folder_path, output_folder, predictor):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for filename in sorted(os.listdir(folder_path)):
        if filename.endswith(".png"):
            image_path = os.path.join(folder_path, filename)
            mask_imagea = process_image(image_path, predictor)

            # 保存掩码
            base_name = os.path.splitext(filename)[0]
            maska_filename = f"mask{base_name}a.png"
            # maskb_filename = f"mask{base_name}b.png"
            mask_imagea_pil = PILImage.fromarray(mask_imagea)
            mask_imagea_pil.save(os.path.join(output_folder, maska_filename))
            # mask_imageb.save(os.path.join(output_folder, maskb_filename))
            print(f"Saved mask for {filename} as {maska_filename}")
            # print(f"Saved mask for {filename} as {maskb_filename}")

class SAM_MASK(Node):
    def __init__(self):
        super().__init__('rgbd_to_mask')

        # 订阅 RGBD 图像
        self.rgbd_sub = self.create_subscription(
            RGBD,
            '/camera/camera/rgbd',  # 订阅的 RGBD 图像话题
            self.rgbd_callback,
            10
        )
        self.get_logger().info("Subscribing to RGBD images")
        self.trigger_sub = self.create_subscription(
            Bool,
            'tmptrigger',  # 订阅的 触发话题
            self.trigger_callback,
            10
        )
        self.get_logger().info("Subscribing to trigger")

        # 发布mask
        self.mask_pub = self.create_publisher(SensorImage, '/tmpmask', 10)
        self.get_logger().info("Publishing tmp mask")

        # CvBridge 实例
        self.bridge = CvBridge()
        self.get_logger().info("CvBridge initialized")
        

        # 初始化变量
        self.rgb_image = None
        self.mask_image = None
        self.status = False
        self.processed_once = False 
        self.predictor = SamPredictor(sam)
        # 创建一个3秒的定时器
        # self.timer = self.create_timer(1.0, self.save_data)
    
    def rgbd_callback(self, msg):
        if not self.status:
            return
        try:
            # 打印实际的编码格式进行调试
            print(f"RGB image encoding: {msg.rgb.encoding}")
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg.rgb, "rgb8")
        except AttributeError as e:
            print(f"AttributeError occurred: {e}")
            # 处理异常情况
        except cv2.error as e:
            print(f"OpenCV error occurred: {e}")
            # 处理异常情况
        
        self.get_logger().info("rgbd image received")
        self.mask = process_image(self.rgb_image, self.predictor, 1)
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(self.mask, "mono8"))
        self.get_logger().info("mask published")
        self.status = False
        self.processed_once = True  # 设置标志位为已处理
        # 销毁节点并关闭 rclpy
        # self.destroy_node()



    def trigger_callback(self, msg):
        self.status = msg.data
        self.get_logger().info("Trigger received")

# 初始化模型

# 批量处理图片
# batch_process_images(input_folder, output_folder, predictor)

def sam4cpp(args=None):
    rclpy.init(args=args)
    rclpy.logging.get_logger('rclpy.executors').set_level(rclpy.logging.LoggingSeverity.WARN)
    node = SAM_MASK()
    rclpy.spin(node)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.processed_once:
            executor.spin_once(timeout_sec=5.0)
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        print("Node has been destroyed and rclpy has been shut down.")

# if __name__ == "__main__":
sam4cpp()