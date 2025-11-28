#ifndef CAR_TYPE_DOUBLE_CONFIRMATION_H
#define CAR_TYPE_DOUBLE_CONFIRMATION_H

#include <open3d/Open3D.h>

struct PointCloudInfo 
{
    // car type of the template point cloud
    std::string car_type = std::string();
    // original template point cloud ptr
    std::shared_ptr<open3d::geometry::PointCloud> pcd_ptr = std::make_shared<open3d::geometry::PointCloud>();
    
    double voxel_size = 9.0;

    // Method 1: Chamfer-Distance
    // kdtree used to search KNN
    std::shared_ptr<open3d::geometry::PointCloud> kdtree_pcd_ptr = std::make_shared<open3d::geometry::PointCloud>();
    std::shared_ptr<open3d::geometry::KDTreeFlann> kdtree_ptr = std::make_shared<open3d::geometry::KDTreeFlann>();
    // average Chamfer-Distance 
    double cd_distance = 0.;

    // Method 2: ICP
    // bounding box of the template point cloud
    Eigen::Vector3d min_bound = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_bound = Eigen::Vector3d::Zero();
    // cropped point cloud ptr by bounding box
    std::shared_ptr<open3d::geometry::PointCloud> pcd_cropped_ptr = std::make_shared<open3d::geometry::PointCloud>();
    // registration result between online point cloud and template point cloud
    open3d::pipelines::registration::RegistrationResult reg_result;
};

class __declspec(dllexport) CarTypeDoubleConfirmation {

public:
    static CarTypeDoubleConfirmation* getInstance();

    /// @brief 全局只初始化一次，勿重复初始化
    /// @param templates_dir given data dir
    /// @return init flag
    bool init(std::string templates_dir);

    /// @brief debug-mode, one-thread version
    std::vector<PointCloudInfo> calcMatchResults(const std::shared_ptr<open3d::geometry::PointCloud>& pcd_online, bool vis_flag = false);
    
    /// @brief core function to match online point cloud with templates, multi-thread version
    /// @param pcd_online online captured point cloud ptr
    /// @return match results sorted by fitness score
    std::vector<PointCloudInfo> calcMatchResultsMultiThreads(const std::shared_ptr<open3d::geometry::PointCloud>& pcd_online, const std::string& carTyp);

private:

    // init flag in case of multiple initialization
    bool _init_flag = false;

    CarTypeDoubleConfirmation() = default;
    CarTypeDoubleConfirmation(const CarTypeDoubleConfirmation&) = delete;
    CarTypeDoubleConfirmation& operator=(const CarTypeDoubleConfirmation&) = delete;

    std::vector<PointCloudInfo> _templates;
};

#endif // CAR_TYPE_DOUBLE_CONFIRMATION_H    
