#ifndef POINT_CLOUD_REGISTRATION_OPEN3D_H
#define POINT_CLOUD_REGISTRATION_OPEN3D_H

#include <open3d/Open3D.h>

class __declspec(dllexport) PointCloudRegistrationOpen3D {

public:
    struct PreprocessOptions {

        bool downsample_flag = true;
        double voxel_size = 5.0;

        bool filter_flag = true;

        // this will be used at radius filter or estimate normals
        int nb_points = 100 / voxel_size;//离群点剔除阈值
        double search_radius = 10.;

        bool estimate_normals_flag = true;
        bool reverse_normals_flag = false;
    };

    struct RegistrationOptions {

        double voxel_size = 5.0;
        double icp_distance_th = voxel_size * + 1.;
        double icp_fitness_th = 0.85;
        int icp_max_iter = 200;
    };

    /// @brief 点云配准外部接口
    /// @param source source点云
    /// @param source_options source点云预处理参数
    /// @param target target点云
    /// @param target_options target点云预处理参数
    /// @param reg_options 配准参数
    /// @param T 输出的最终变换矩阵T_target_source
    /// @param vis_flag 是否可视化
    /// @return 配准结果
    open3d::pipelines::registration::RegistrationResult PCDRegistration(std::shared_ptr<open3d::geometry::PointCloud>& source, const PreprocessOptions& source_options,
                                std::shared_ptr<open3d::geometry::PointCloud>& target, const PreprocessOptions& target_options,
                                const RegistrationOptions& reg_options, Eigen::Matrix4d& T, bool vis_flag);

    PointCloudRegistrationOpen3D() = default;
  
private:
    /// @brief 可视化配准结果
    /// @param source source点云
    /// @param target target点云
    /// @param pairs 配准点对
    /// @param T 变换矩阵T_target_source
    /// @param win_name 窗口名称
    void draw_registration_result(std::shared_ptr<open3d::geometry::PointCloud>& source, std::shared_ptr<open3d::geometry::PointCloud>& target,
        const open3d::pipelines::registration::CorrespondenceSet& pairs, const Eigen::Matrix4d& T, std::string win_name = "");

    /// @brief 点云预处理
    /// @param pcd 输入的点云
    /// @param pcd_final output处理后的点云
    /// @param pcd_fpfh output处理后的点云的FPFH特征
    /// @param show_outliers 是否可视化野点(用于Debug)
    /// @param win_name 可视化窗口名称
    /// @return 是否预处理成功
    bool PCDPreprocess(std::shared_ptr<open3d::geometry::PointCloud>& pcd, const PreprocessOptions& options,
        std::shared_ptr<open3d::geometry::PointCloud>& pcd_final, std::shared_ptr<open3d::pipelines::registration::Feature>& pcd_fpfh,
        bool show_outliers = false, std::string win_name = "");

    /// @brief 由粗到精的点云配准
    /// @param source 源点云
    /// @param target 目标点云
    /// @param source_fpfh 源点云的FPFH特征
    /// @param target_fpfh 目标点云的FPFH特征
    /// @param voxel_size 点云预处理使用的体素参数
    /// @param vis_flag 是否可视化(用于Debug)
    /// @return 配准结果
    open3d::pipelines::registration::RegistrationResult PairRegistration(std::shared_ptr<open3d::geometry::PointCloud>& source, std::shared_ptr<open3d::geometry::PointCloud>& target,
        std::shared_ptr<open3d::pipelines::registration::Feature>& source_fpfh, std::shared_ptr<open3d::pipelines::registration::Feature>& target_fpfh,
        const RegistrationOptions& reg_options, bool vis_flag);
};

#endif // !POINT_CLOUD_REGISTRATION_OPEN3D_H
