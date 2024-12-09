# import cv2
# import numpy as np

# # 读取图像
# img1 = cv2.imread('2.jpg')
# img2 = cv2.imread('1.jpg')

# # 转换为灰度图像（SIFT 也可以直接处理彩色图像，但这里为了统一处理，转换为灰度图像）
# gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
# gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

# # 创建 SIFT 检测器
# sift = cv2.SIFT_create()

# # 计算关键点和描述符
# kp1, des1 = sift.detectAndCompute(gray1, None)
# kp2, des2 = sift.detectAndCompute(gray2, None)

# # 使用 BFMatcher 进行特征匹配
# bf = cv2.BFMatcher()
# matches = bf.knnMatch(des1, des2, k=2)

# # 应用比例测试来筛选匹配点
# good_matches = []
# for m, n in matches:
#     if m.distance < 0.75 * n.distance:
#         good_matches.append(m)

# # 获取匹配点的坐标
# src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
# dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

# # 使用 RANSAC 计算单应性矩阵
# H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

# # 应用单应性矩阵进行透视变换
# height, width, _ = img2.shape
# img1_transformed = cv2.warpPerspective(img1, H, (width, height))

# # 绘制匹配结果
# img_matches = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# # # 显示匹配结果
# # cv2.imshow("Matches", img_matches)
# # cv2.waitKey(0)
# # cv2.destroyAllWindows()

# # 显示变换后的图像
# cv2.imshow("Transformed Image", img1_transformed)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# import cv2
# import numpy as np

# def convert_to_hsv(image):
#     """
#     将图像从 BGR 格式转换为 HSV 格式。
#     """
#     hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#     return hsv_image

# def extract_hsv_sift_descriptors(image):
#     """
#     提取图像的 HSV-SIFT 描述符。
#     """
#     # 转换为 HSV 图像
#     hsv_image = convert_to_hsv(image)
    
#     # 分离 HSV 通道
#     h, s, v = cv2.split(hsv_image)
    
#     # 创建 SIFT 检测器
#     sift = cv2.SIFT_create()
    
#     # 计算关键点和描述符
#     kp, des = sift.detectAndCompute(v, None)
    
#     # 将关键点的位置映射回原图
#     keypoints = [cv2.KeyPoint(x=kp.pt[0], y=kp.pt[1], size=kp.size, angle=kp.angle, response=kp.response, octave=kp.octave, class_id=kp.class_id) for kp in kp]
    
#     # 提取 HSV 通道的值
#     hsv_values = []
#     for kp in keypoints:
#         x, y = int(kp.pt[0]), int(kp.pt[1])
#         h_val = h[y, x]
#         s_val = s[y, x]
#         v_val = v[y, x]
#         hsv_values.append((h_val, s_val, v_val))
    
#     # 将 HSV 值与 SIFT 描述符结合
#     hsv_sift_descriptors = []
#     for i, kp in enumerate(keypoints):
#         hsv_sift_descriptor = np.concatenate((des[i], np.array(hsv_values[i])))
#         hsv_sift_descriptors.append(hsv_sift_descriptor)
    
#     return keypoints, np.array(hsv_sift_descriptors)

# def match_images(image1, image2):
#     """
#     使用 HSV-SIFT 描述符匹配两张图像。
#     """
#     # 提取两张图像的 HSV-SIFT 描述符
#     keypoints1, descriptors1 = extract_hsv_sift_descriptors(image1)
#     keypoints2, descriptors2 = extract_hsv_sift_descriptors(image2)
    
#     # 检查描述符是否为空
#     if descriptors1 is None or descriptors2 is None:
#         print("无法提取描述符。")
#         return None, None, None
    
#     # 使用 BFMatcher 进行描述符匹配
#     bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
#     matches = bf.match(descriptors1, descriptors2)
    
#     # 按照匹配距离排序
#     matches = sorted(matches, key=lambda x: x.distance)
    
#     return keypoints1, keypoints2, matches

# # 读取两张图片
# image1 = cv2.imread('2.jpg')
# image2 = cv2.imread('1.jpg')

# # 执行匹配
# keypoints1, keypoints2, matches = match_images(image1, image2)

# if matches is not None:
#     # 获取匹配点的坐标
#     src_pts = np.float32([keypoints1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
#     dst_pts = np.float32([keypoints2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    
#     # 使用 RANSAC 计算单应性矩阵
#     H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    
#     # 应用单应性矩阵进行透视变换
#     height, width, _ = image2.shape
#     img1_transformed = cv2.warpPerspective(image1, H, (width, height))
    
#     # 将原图和变换后的图像水平拼接在一起
    
#     combined_image = cv2.hconcat([image2, img1_transformed])
#     scale_factor = 0.8  # 调整为原来的 80%
#     new_width = int(combined_image.shape[1] * scale_factor)
#     new_height = int(combined_image.shape[0] * scale_factor)
#     combined_image = cv2.resize(combined_image, (new_width, new_height))
    
#     # 显示拼接后的图像
#     print("Homography Matrix:\n", H)
#     cv2.imshow('Original and Transformed Image', combined_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
    
#     # 打印单应性矩阵
   
# else:
#     print("无法找到足够的匹配点。")

import cv2
import numpy as np

# Define the camera intrinsic matrix (example values, adjust accordingly)
fx = 800  # Focal length in pixels (x)
fy = 800  # Focal length in pixels (y)
cx = 640  # Principal point (x)
cy = 480  # Principal point (y)

# Camera intrinsic matrix (3x3)
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]])

# Distortion coefficients (assuming no distortion here)
dist_coeffs = np.zeros((4, 1))

# Example 3D object points (should match your 3D model of the object)
# These 3D points are in the object coordinate frame
object_points = np.array([
    [0.0, 0.0, 0.0],  # Example 3D point 1
    [1.0, 0.0, 0.0],  # Example 3D point 2
    [0.0, 1.0, 0.0],  # Example 3D point 3
    [1.0, 1.0, 0.0],  # Example 3D point 4
], dtype=np.float32)

# Load the image with the object (replace with your image path)
image = cv2.imread("object_image.jpg")

# Step 1: Detect SIFT features in the image
sift = cv2.SIFT_create()
keypoints, descriptors = sift.detectAndCompute(image, None)

# Step 2: Load the 2D points in the image that correspond to the object model's 3D points
# For now, let's assume these points are manually selected or detected via some method
# These 2D points must correspond to the 3D object points (in image coordinates)
image_points = np.array([
    [320, 240],  # Corresponding 2D point 1 (manually selected or detected)
    [420, 240],  # Corresponding 2D point 2
    [320, 340],  # Corresponding 2D point 3
    [420, 340],  # Corresponding 2D point 4
], dtype=np.float32)

# Step 3: Solve PnP to find the object's pose relative to the camera
# Use cv2.solvePnP to compute the rotation and translation vectors
success, rotation_vector, translation_vector = cv2.solvePnP(
    object_points,  # 3D points in object space
    image_points,   # 2D points in image space
    K,              # Camera intrinsic matrix
    dist_coeffs     # Distortion coefficients
)

# Check if PnP was successful
if success:
    print("PnP solved successfully!")
    print("Rotation vector:\n", rotation_vector)
    print("Translation vector:\n", translation_vector)
else:
    print("PnP failed to solve!")

# Step 4: Optionally, project the 3D points back into the image to visualize
# This helps to validate if the pose estimation is correct
projected_points, _ = cv2.projectPoints(object_points, rotation_vector, translation_vector, K, dist_coeffs)

# Draw the projected points on the image
for point in projected_points:
    x, y = point[0]
    cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)

# Display the image with projected points
cv2.imshow("Image with Projected Points", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
