cmake_minimum_required(VERSION 3.10)

project(rgbdp_info)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/opencv4")
include_directories(${PCL_INCLUDE_DIRS} )

# 添加可执行文件
add_executable(rgbdp_info_node src/rgbdp_info.cpp)

target_include_directories(rgbdp_info_node PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(rgbdp_info_node
    /usr/lib/x86_64-linux-gnu/libopencv_stitching.so
    /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so
    /usr/lib/x86_64-linux-gnu/libopencv_aruco.so
    /usr/lib/x86_64-linux-gnu/libopencv_barcode.so
    /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so
    /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so
    /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so
    /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so
    /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so
    /usr/lib/x86_64-linux-gnu/libopencv_dpm.so
    /usr/lib/x86_64-linux-gnu/libopencv_face.so
    /usr/lib/x86_64-linux-gnu/libopencv_freetype.so
    /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so
    /usr/lib/x86_64-linux-gnu/libopencv_hdf.so
    /usr/lib/x86_64-linux-gnu/libopencv_hfs.so
    /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so
    /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so
    /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so
    /usr/lib/x86_64-linux-gnu/libopencv_mcc.so
    /usr/lib/x86_64-linux-gnu/libopencv_quality.so
    /usr/lib/x86_64-linux-gnu/libopencv_rapid.so
    /usr/lib/x86_64-linux-gnu/libopencv_reg.so
    /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so
    /usr/lib/x86_64-linux-gnu/libopencv_saliency.so
    /usr/lib/x86_64-linux-gnu/libopencv_shape.so
    /usr/lib/x86_64-linux-gnu/libopencv_stereo.so
    /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so
    /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so
    /usr/lib/x86_64-linux-gnu/libopencv_superres.so
    /usr/lib/x86_64-linux-gnu/libopencv_optflow.so
    /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so
    /usr/lib/x86_64-linux-gnu/libopencv_tracking.so
    /usr/lib/x86_64-linux-gnu/libopencv_highgui.so
    /usr/lib/x86_64-linux-gnu/libopencv_datasets.so
    /usr/lib/x86_64-linux-gnu/libopencv_text.so
    /usr/lib/x86_64-linux-gnu/libopencv_plot.so
    /usr/lib/x86_64-linux-gnu/libopencv_ml.so
    /usr/lib/x86_64-linux-gnu/libopencv_videostab.so
    /usr/lib/x86_64-linux-gnu/libopencv_videoio.so
    /usr/lib/x86_64-linux-gnu/libopencv_viz.so
    /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so
    /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so
    /usr/lib/x86_64-linux-gnu/libopencv_video.so
    /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so
    /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so
    /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so
    /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so
    /usr/lib/x86_64-linux-gnu/libopencv_features2d.so
    /usr/lib/x86_64-linux-gnu/libopencv_dnn.so
    /usr/lib/x86_64-linux-gnu/libopencv_flann.so
    /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so
    /usr/lib/x86_64-linux-gnu/libopencv_photo.so
    /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so
    /usr/lib/x86_64-linux-gnu/libopencv_core.so
)
# 链接OpenCV库


# 添加依赖
ament_target_dependencies(rgbdp_info_node
    rclcpp
    cv_bridge
    image_transport
    sensor_msgs
    realsense2_camera_msgs
    PCL 
    pcl_conversions
)

# 安装目标
install(TARGETS
    rgbdp_info_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
