#include "PointCloudRegistrationOpen3D.h"
#include "speedlog.h"
#include "tic_toc.h"


open3d::pipelines::registration::RegistrationResult PointCloudRegistrationOpen3D::PCDRegistration(std::shared_ptr<open3d::geometry::PointCloud>& source, const PreprocessOptions& source_options,
                                            std::shared_ptr<open3d::geometry::PointCloud>& target, const PreprocessOptions& target_options,
                                            const RegistrationOptions& reg_options, Eigen::Matrix4d& T, bool vis_flag) {
    std::shared_ptr<open3d::geometry::PointCloud> source_down = std::make_shared<open3d::geometry::PointCloud>();
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh = std::make_shared<open3d::pipelines::registration::Feature>();

    std::shared_ptr<open3d::geometry::PointCloud> target_down = std::make_shared<open3d::geometry::PointCloud>();
    std::shared_ptr<open3d::pipelines::registration::Feature> target_fpfh = std::make_shared<open3d::pipelines::registration::Feature>();

    LOG_INFO("Starting PCD Registration......");
    LOG_INFO("Preprocessing PCD Source & Target...\n");
      
    TicToc tcnt;
    if (!PCDPreprocess(source, source_options, source_down, source_fpfh, vis_flag, "Source"))
    {
        LOG_ERROR("Failed to Preprocess Source PCD!");
        return open3d::pipelines::registration::RegistrationResult();
    }
    LOG_INFO("Preprocess Source PCD Done!\n");

    if (!PCDPreprocess(target, target_options, target_down, target_fpfh, vis_flag, "Target"))
    {
        LOG_ERROR("Failed to Preprocess Target PCD!");
        return open3d::pipelines::registration::RegistrationResult();
    }
    LOG_INFO("Preprocess Target PCD Done!\n");

    LOG_INFO("PCD Preprocess Result:");
    LOG_DEBUG("Source Points:\t{} ---> {}", source->points_.size(), source_down->points_.size());
    LOG_DEBUG("Target Points:\t{} ---> {}", target->points_.size(), target_down->points_.size());

    LOG_INFO("Starting Registration......\n");
    open3d::pipelines::registration::RegistrationResult reg_result = PairRegistration(source_down, target_down, source_fpfh, target_fpfh, reg_options, vis_flag);
    LOG_INFO("Registration Total Time: {}s.", tcnt.toc());
   
    return reg_result;
}


void PointCloudRegistrationOpen3D::draw_registration_result(std::shared_ptr<open3d::geometry::PointCloud>& source, std::shared_ptr<open3d::geometry::PointCloud>& target,
    const open3d::pipelines::registration::CorrespondenceSet& pairs, const Eigen::Matrix4d& T, std::string win_name) {

    std::shared_ptr<open3d::geometry::PointCloud> source_temp = std::make_shared<open3d::geometry::PointCloud>(*source);
    std::shared_ptr<open3d::geometry::PointCloud> target_temp = std::make_shared<open3d::geometry::PointCloud>(*target);
    source_temp->PaintUniformColor(Eigen::Vector3d(1, 0.706, 0));
    target_temp->PaintUniformColor(Eigen::Vector3d(0, 0.651, 0.929));
    //for (const auto& pair : pairs) {
    //    source_temp->colors_[pair[0]] = Eigen::Vector3d(1, 0, 0);
    //    target_temp->colors_[pair[1]] = Eigen::Vector3d(1, 0, 0);
    //}

    source_temp->Transform(T);
    open3d::visualization::DrawGeometries({ source_temp, target_temp, open3d::geometry::TriangleMesh::CreateCoordinateFrame(100) }, win_name);
}

bool PointCloudRegistrationOpen3D::PCDPreprocess(std::shared_ptr<open3d::geometry::PointCloud>& pcd, const PreprocessOptions& options,
    std::shared_ptr<open3d::geometry::PointCloud>& pcd_final, std::shared_ptr<open3d::pipelines::registration::Feature>& pcd_fpfh, 
    bool show_outliers, std::string win_name) {
    
    TicToc tcnt;
    auto pcd_down = pcd;
    if (options.downsample_flag) {
        // 下采样并计算法向量
        LOG_INFO("Voxel Down Sample...");
        pcd_down = pcd->VoxelDownSample(options.voxel_size);
    }

    if (options.filter_flag) {
        // 滤波
        LOG_INFO("Filtering...");
        auto pcd_filtered_tuple = pcd_down->RemoveRadiusOutliers(options.nb_points, options.search_radius, false);
        pcd_final = std::get<0>(pcd_filtered_tuple);
        LOG_DEBUG("{} Outliers Filtered.", pcd_down->points_.size() - pcd_final->points_.size());

        if (show_outliers) {
            pcd_down->PaintUniformColor(Eigen::Vector3d(0, 0, 0));
            for (auto idx : std::get<1>(pcd_filtered_tuple))
                pcd_down->colors_[idx] = Eigen::Vector3d(255, 0, 0);
            open3d::visualization::DrawGeometries({ pcd_down, open3d::geometry::TriangleMesh::CreateCoordinateFrame(100) }, win_name);
        }
    }
    else {
        pcd_final = pcd_down;
    }
    
    if (pcd_final->points_.size() < 5000) {
        LOG_ERROR("Fatal Error: Processed PCD Points: {} < 5000!", pcd_final->points_.size());
        return false;
    }

    
    if (options.estimate_normals_flag) {
        LOG_INFO("Estimate Normals...");
        pcd_final->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(options.search_radius, options.nb_points));
        if (!pcd_final->HasNormals()) {
            LOG_ERROR("Failed to Estimate Normals!");
            return false;
        }

        pcd_final->OrientNormalsTowardsCameraLocation();
        if (options.reverse_normals_flag) {
            for (auto& n : pcd_final->normals_) {
                n *= -1;
            }
        }
    }

    /*
    // 计算FPFH特征
    LOG_INFO("Compute FPFH Feature...");
    pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pcd_final, open3d::geometry::KDTreeSearchParamHybrid(options.search_radius, options.nb_points));
    */

    LOG_INFO("PCD Preprocessing Time:\t{} s.", tcnt.toc());
    
    return true;
}

open3d::pipelines::registration::RegistrationResult PointCloudRegistrationOpen3D::PairRegistration(std::shared_ptr<open3d::geometry::PointCloud>& source, std::shared_ptr<open3d::geometry::PointCloud>& target,
    std::shared_ptr<open3d::pipelines::registration::Feature>& source_fpfh, std::shared_ptr<open3d::pipelines::registration::Feature>& target_fpfh,
    const RegistrationOptions& reg_options, bool vis_flag) {

    LOG_INFO("Step 1: Fast Global Registration");
    TicToc tcnt;
    
    /*
    open3d::pipelines::registration::FastGlobalRegistrationOption fgr_option;
    fgr_option.maximum_correspondence_distance_ = reg_options.distance_threshold_fgr;
    fgr_option.iteration_number_ = reg_options.iteration_num;
    fgr_option.tuple_test_ = reg_options.tuple_test;

    auto fgr_result = open3d::pipelines::registration::FastGlobalRegistrationBasedOnFeatureMatching(*source, *target,
        *source_fpfh, *target_fpfh, fgr_option);
    LOG_INFO(this->_logger, "FGR Time:\t{} s.", tcnt.toc());
    LOG_DEBUG(this->_logger, "FRG Fitness of Source: {}%", fgr_result.fitness_ * 100.);

    if (vis_flag) {
        std::string win_name = "FGR Pairs Num: " + std::to_string(fgr_result.correspondence_set_.size())
            + " Fitness of Source: " + std::to_string(fgr_result.fitness_)
            + " FGR Time: " + std::to_string(tcnt.toc()) + " s.";
        draw_registration_result(source, target, fgr_result.correspondence_set_, fgr_result.transformation_, win_name);
    }
    */
    
    /*
    auto coarse_result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(*source, *target, *source_fpfh, *target_fpfh, true, 1,
        open3d::pipelines::registration::TransformationEstimationPointToPoint(), 10);
    if (vis_flag) {
        std::string win_name = "RANSAC Pairs Num: " + std::to_string(coarse_result.correspondence_set_.size())
            + " Fitness of Source: " + std::to_string(coarse_result.fitness_)
            + " RANSAC Time: " + std::to_string(tcnt.toc()) + " s.";
        draw_registration_result(source, target, coarse_result.correspondence_set_, coarse_result.transformation_, win_name);
    }
    LOG_INFO(this->_logger, "RANSAC Time:\t{} s.", tcnt.toc());
    LOG_DEBUG(this->_logger, "RANSAC Fitness of Source: {}%", coarse_result.fitness_ * 100.);
    */
    
    LOG_INFO("Step 2: ICP Registration");
    tcnt.tic();
    auto icp_result = open3d::pipelines::registration::RegistrationICP(*source, *target, reg_options.icp_distance_th,
        Eigen::Matrix4d::Identity(),
        open3d::pipelines::registration::TransformationEstimationPointToPoint(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, reg_options.icp_max_iter));
    LOG_INFO("ICP Time:\t{} s.", tcnt.toc());
    LOG_DEBUG("ICP Fitness of Source: {}%", icp_result.fitness_ * 100.);

    if (vis_flag) {
        std::string win_name = "ICP Pairs Num: " + std::to_string(icp_result.correspondence_set_.size()) + " Fitness of Source: " + std::to_string(icp_result.fitness_);
        draw_registration_result(source, target, icp_result.correspondence_set_, icp_result.transformation_, win_name);
    }
    std::stringstream ss;
    ss << icp_result.transformation_;
    LOG_DEBUG("\nICP Result:\n{}", ss.str());
    LOG_DEBUG("ICP Inlier Num:\t{}", icp_result.correspondence_set_.size());
    LOG_DEBUG("ICP RMSE:\t{}", icp_result.inlier_rmse_ / reg_options.voxel_size);

    return icp_result;
}
