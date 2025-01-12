#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/ColoredICP.h>
#include <chrono>

int main() {
    // 读取源点云和目标点云
        // 创建一个读取点云的选项对象
    open3d::io::ReadPointCloudOption params;
    // 设置读取的选项
    params.remove_infinite_points = true;    
    params.remove_nan_points = true;     
    params.print_progress = true; 
    auto source = new(open3d::geometry::PointCloud);
    auto target = new(open3d::geometry::PointCloud);
    open3d::io::ReadPointCloudFromPCD("/home/ubuntu/step2pcd/cube1.pcd",*source,params);
    open3d::io::ReadPointCloudFromPCD("/home/ubuntu/step2pcd/t2.pcd",*target,params);

    // 体素尺寸 (根据需要进行调整)
    double voxel_size = 0.05;

    // 初始变换矩阵，通常是单位矩阵
    Eigen::Matrix4d init_trans = Eigen::Matrix4d::Identity();
    init_trans<< 0.35355339, -0.5732233,  0.73919892, 5.07,
                      0.61237244,  0.73919892,  0.28033009, -3.07,
                      -0.70710678, 0.35355339,  0.61237244, 10.08,
                      0.0,          0.0,          0.0,          1.0;

        // 计算源点云法线
    auto start = std::chrono::high_resolution_clock::now();
    source->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
    // 计算目标点云法线
    target->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));

    // 使用 Colored ICP 算法
    auto result = open3d::pipelines::registration::RegistrationColoredICP(
        *source, *target, voxel_size, init_trans,open3d::pipelines::registration::TransformationEstimationForColoredICP(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6,30)
    );

    // 输出变换矩阵
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Final Transformation: \n" << result.transformation_<<std::endl;
    std::cout << "Elapsed time: " << elapsed.count() << " seconds" << std::endl;
    // 可视化配准结果
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries;
    source->Transform(result.transformation_);
    geometries.push_back(std::make_shared<open3d::geometry::PointCloud>(*source));
    geometries.push_back(std::make_shared<open3d::geometry::PointCloud>(*target));

    // 使用 DrawGeometries 来绘制源和目标点云
    open3d::visualization::DrawGeometries(geometries, "Colored ICP Registration");


    return 0;
}
