cmake_minimum_required(VERSION 3.20)
project(color_icp)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(VTK REQUIRED)  # 添加这一行
find_package(rcl_interfaces REQUIRED)  # 添加这一行
# find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)




include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}  # 添加这一行
    # ${OpenCV_INCLUDE_DIRS}
)

add_executable(color_icp_node src/color_icp.cpp)
# target_link_libraries(color_icp_node PRIVATE PCL sensor_msgs rclcpp pcl_conversions ${VTK_LIBRARIES} rcl_interfaces)  # 添加这一行

add_executable(optimization src/optimization.cpp)
# target_link_libraries(optimization PRIVATE PCL ${VTK_LIBRARIES} rcl_interfaces)  # 添加这一行
target_link_libraries(color_icp_node PRIVATE
    ${PCL_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
    ${VTK_LIBRARIES}
    ${rcl_interfaces_LIBRARIES}
    # /usr/lib/x86_64-linux-gnu/libopencv_stitching.so
    # /usr/lib/x86_64-linux-gnu/libopencv_alphamat.so
    # /usr/lib/x86_64-linux-gnu/libopencv_aruco.so
    # /usr/lib/x86_64-linux-gnu/libopencv_barcode.so
    # /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so
    # /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so
    # /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so
    # /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so
    # /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so
    # /usr/lib/x86_64-linux-gnu/libopencv_dpm.so
    # /usr/lib/x86_64-linux-gnu/libopencv_face.so
    # /usr/lib/x86_64-linux-gnu/libopencv_freetype.so
    # /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so
    # /usr/lib/x86_64-linux-gnu/libopencv_hdf.so
    # /usr/lib/x86_64-linux-gnu/libopencv_hfs.so
    # /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so
    # /usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so
    # /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so
    # /usr/lib/x86_64-linux-gnu/libopencv_mcc.so
    # /usr/lib/x86_64-linux-gnu/libopencv_quality.so
    # /usr/lib/x86_64-linux-gnu/libopencv_rapid.so
    # /usr/lib/x86_64-linux-gnu/libopencv_reg.so
    # /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so
    # /usr/lib/x86_64-linux-gnu/libopencv_saliency.so
    # /usr/lib/x86_64-linux-gnu/libopencv_shape.so
    # /usr/lib/x86_64-linux-gnu/libopencv_stereo.so
    # /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so
    # /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so
    # /usr/lib/x86_64-linux-gnu/libopencv_superres.so
    # /usr/lib/x86_64-linux-gnu/libopencv_optflow.so
    # /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so
    # /usr/lib/x86_64-linux-gnu/libopencv_tracking.so
    # /usr/lib/x86_64-linux-gnu/libopencv_highgui.so
    # /usr/lib/x86_64-linux-gnu/libopencv_datasets.so
    # /usr/lib/x86_64-linux-gnu/libopencv_text.so
    # /usr/lib/x86_64-linux-gnu/libopencv_plot.so
    # /usr/lib/x86_64-linux-gnu/libopencv_ml.so
    # /usr/lib/x86_64-linux-gnu/libopencv_videostab.so
    # /usr/lib/x86_64-linux-gnu/libopencv_videoio.so
    # /usr/lib/x86_64-linux-gnu/libopencv_viz.so
    # /usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so
    # /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so
    # /usr/lib/x86_64-linux-gnu/libopencv_video.so
    # /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so
    # /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so
    # /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so
    # /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so
    # /usr/lib/x86_64-linux-gnu/libopencv_features2d.so
    # /usr/lib/x86_64-linux-gnu/libopencv_dnn.so
    # /usr/lib/x86_64-linux-gnu/libopencv_flann.so
    # /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so
    # /usr/lib/x86_64-linux-gnu/libopencv_photo.so
    # /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so
    # /usr/lib/x86_64-linux-gnu/libopencv_core.so 

)

target_link_libraries(optimization PRIVATE
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    ${rcl_interfaces_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    color_icp_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

install(FILES package.xml
    DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(
    INSTALL_TO_SHARE
    config
    launch

)