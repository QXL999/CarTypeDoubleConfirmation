#include "CarTypeDoubleConfirmation.h"
#include <filesystem>
#include <opencv2/core.hpp>
#include "PointCloudRegistrationOpen3D.h"
#include <future>
#include "speedlog.h"

namespace fs = std::filesystem;

CarTypeDoubleConfirmation* CarTypeDoubleConfirmation::getInstance() {
    static CarTypeDoubleConfirmation instance;
    return &instance;
}

bool CarTypeDoubleConfirmation::init(std::string templates_dir) {

    SpeedLogger::LoggerConfig logConfig("./Log_CarTypeDoubleConfirmationMultiThreads/", "Log_Init");
    SpeedLogger::getInstance()->init(logConfig);


    if (_init_flag) {
        LOG_INFO("Already initialized");
        SpeedLogger::getInstance()->unInit();
        return true;
    }

    if (!fs::exists(templates_dir)) {
        LOG_ERROR("Templates directory does not exist -> {}", templates_dir);
        SpeedLogger::getInstance()->unInit();
        return false;
    }

    _templates.clear();
    LOG_INFO("Template Vector Cleared.");

    for (const auto& entry : fs::directory_iterator(templates_dir)) {
        if (entry.is_directory()) {
            std::string car_type_dir = entry.path().string();
            std::string car_type = entry.path().filename().string();
            LOG_INFO("Car Type Dir: {}", car_type_dir);
            LOG_INFO("Car Type: {}", car_type);
            std::cout<<"Car Type Dir: "<< car_type_dir <<std::endl;
            // Step 1: Load Original Template Point Cloud
            std::string template_pcd_path = car_type_dir + "\\" + car_type + ".ply";
            LOG_INFO("Trying to Load Template: {}", template_pcd_path);
            if (!fs::exists(template_pcd_path)) {
                LOG_ERROR("Template file not exists: {}", template_pcd_path);
                SpeedLogger::getInstance()->unInit();
                return false;
            }
            auto template_pcd_ptr = std::make_shared<open3d::geometry::PointCloud>();
            open3d::io::ReadPointCloud(template_pcd_path, *template_pcd_ptr);
            if (template_pcd_ptr->IsEmpty()) 
            {
                LOG_ERROR(" Template Point Cloud is Empty -> {}", template_pcd_path);
                SpeedLogger::getInstance()->unInit();
                return false;
            }
            LOG_INFO("Templatet Point Cloud Successfully Loaded.");

            PointCloudInfo pointCloudInfo;

            LOG_INFO("Trying to Set KDTree...");
            pointCloudInfo.kdtree_pcd_ptr = std::make_shared<open3d::geometry::PointCloud>(*template_pcd_ptr);
            pointCloudInfo.kdtree_pcd_ptr = pointCloudInfo.kdtree_pcd_ptr->VoxelDownSample(pointCloudInfo.voxel_size);
            pointCloudInfo.kdtree_ptr->SetGeometry(*pointCloudInfo.kdtree_pcd_ptr);
            LOG_INFO("Set KDTree Successfully.");


            // Step 2: Load Bounding Box
            std::string roi_path = car_type_dir + "\\ROI.yml";
            LOG_INFO("Trying to Load ROI -> {}", roi_path);
            cv::FileStorage fs(roi_path, cv::FileStorage::READ);
            if (!fs.isOpened()) {
                LOG_ERROR("Failed to Load ROI.");
                SpeedLogger::getInstance()->unInit();
                return false;
            }
            double min_x, min_y, min_z, max_x, max_y, max_z;
            fs["min_bound_x"] >> min_x;
            fs["min_bound_y"] >> min_y;
            fs["min_bound_z"] >> min_z;
            fs["max_bound_x"] >> max_x;
            fs["max_bound_y"] >> max_y;
            fs["max_bound_z"] >> max_z;
            fs.release();
            LOG_INFO("ROI Loaded.");

            pointCloudInfo.car_type = car_type;
            LOG_INFO("Car Type Loaded.");
            pointCloudInfo.pcd_ptr = std::make_shared<open3d::geometry::PointCloud>(*template_pcd_ptr);  // original template point cloud
            pointCloudInfo.pcd_ptr = pointCloudInfo.pcd_ptr->VoxelDownSample(pointCloudInfo.voxel_size);
            LOG_INFO("Template Point Cloud Loaded.");

            pointCloudInfo.min_bound = Eigen::Vector3d(min_x, min_y, min_z);
            pointCloudInfo.max_bound = Eigen::Vector3d(max_x, max_y, max_z);
            LOG_INFO("min_bound: {}", pointCloudInfo.min_bound.transpose());
            LOG_INFO("max_bound: {}", pointCloudInfo.max_bound.transpose());
            LOG_INFO("ROI Bounding Box Loaded.");
            LOG_INFO("Point Cloud Size Before Crop: {}", pointCloudInfo.pcd_ptr->points_.size());
            pointCloudInfo.pcd_cropped_ptr = pointCloudInfo.pcd_ptr->Crop(open3d::geometry::AxisAlignedBoundingBox(
                pointCloudInfo.min_bound, pointCloudInfo.max_bound));
            LOG_INFO("Point Cloud Size After Crop: {}", pointCloudInfo.pcd_cropped_ptr->points_.size());
            LOG_INFO("PCD Cropped by ROI Bounding Box.");
            LOG_INFO("DownSampled Point Cloud Size: {}", pointCloudInfo.pcd_ptr->points_.size());
            LOG_INFO("Cropped Point Cloud Size: {}", pointCloudInfo.pcd_cropped_ptr->points_.size());
            LOG_INFO("Template of {} Loaded.\n", car_type);
            _templates.push_back(pointCloudInfo);
        }
    }
    SpeedLogger::getInstance()->unInit();
    _init_flag = true;
    return true;
}

std::vector<PointCloudInfo> CarTypeDoubleConfirmation::calcMatchResults(const std::shared_ptr<open3d::geometry::PointCloud>& pcd_online, bool vis_flag) {

    SpeedLogger::LoggerConfig logConfig("./Log_CarTypeDoubleConfirmation/", "Log_Algo");
    SpeedLogger::getInstance()->init(logConfig);

    for (auto& template_pcd_info : _templates) {
        // reset
        template_pcd_info.reg_result = open3d::pipelines::registration::RegistrationResult();
    }


    auto pcd_online_down = pcd_online->VoxelDownSample(_templates[0].voxel_size);


    for (auto& template_pcd_info : _templates) {
        LOG_DEBUG("Matching with {}...", template_pcd_info.car_type);

        auto pcd_online_croped = pcd_online_down->Crop(open3d::geometry::AxisAlignedBoundingBox(
            template_pcd_info.min_bound, template_pcd_info.max_bound));

        // ICP
        PointCloudRegistrationOpen3D::PreprocessOptions source_optins;
        source_optins.downsample_flag = false;  // already downsampled when init
        source_optins.voxel_size = template_pcd_info.voxel_size;
        source_optins.filter_flag = false;
        source_optins.estimate_normals_flag = false;


        PointCloudRegistrationOpen3D::PreprocessOptions target_options;
        target_options.downsample_flag = false;  // already downsampled when init
        target_options.voxel_size = template_pcd_info.voxel_size;
        target_options.filter_flag = false;
        target_options.estimate_normals_flag = false;

        PointCloudRegistrationOpen3D::RegistrationOptions reg_options;
        reg_options.voxel_size = template_pcd_info.voxel_size;
        reg_options.icp_distance_th = template_pcd_info.voxel_size + 2;

        Eigen::Matrix4d T_target_source = Eigen::Matrix4d::Identity();
        bool vis_flag = true;
        PointCloudRegistrationOpen3D reg;
        auto reg_result = reg.PCDRegistration(pcd_online_croped, source_optins, template_pcd_info.pcd_cropped_ptr, target_options,
            reg_options, T_target_source, vis_flag);

        template_pcd_info.reg_result = reg_result;
    
    
        LOG_DEBUG("CD: Matching with {}...", template_pcd_info.car_type);
        auto t1 = std::chrono::high_resolution_clock::now();
        double distAB = 0.0;
#pragma omp parallel for
        for (const auto& point : pcd_online_down->points_) {
            std::vector<int> indices(1);
            std::vector<double> dists(1);
            if (template_pcd_info.kdtree_ptr->SearchKNN(point, 1, indices, dists) > 0) {
                distAB += dists[0];
            }
        }
        template_pcd_info.cd_distance = distAB / pcd_online_down->points_.size();
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() * 0.001;
        LOG_DEBUG("CD Calculation Time: {} seconds", duration);

    }


    std::sort(_templates.begin(), _templates.end(), [](const PointCloudInfo& a, const PointCloudInfo& b) {
        return a.reg_result.fitness_ > b.reg_result.fitness_;
        });

std::stringstream ss;
for (const auto& template_pcd_info : _templates) {
    ss << "\nMatch Result with " << template_pcd_info.car_type << ":\n"
        << "  Fitness:\t" << template_pcd_info.reg_result.fitness_ << std::endl
        << "  CD Distance:\t" << template_pcd_info.cd_distance << std::endl;
}
LOG_DEBUG(ss.str());
SpeedLogger::getInstance()->unInit();
return _templates;

}

std::vector<PointCloudInfo> CarTypeDoubleConfirmation::calcMatchResultsMultiThreads(const std::shared_ptr<open3d::geometry::PointCloud>& pcd_online, const std::string& carType) {

    SpeedLogger::LoggerConfig logConfig("./Log_CarTypeDoubleConfirmationMultiThreads/", "Log_Algo_" + carType);
    SpeedLogger::getInstance()->init(logConfig);
    bool IsCarType = false;
    for (auto& template_pcd_info : _templates)
    {
        // reset
        template_pcd_info.reg_result = open3d::pipelines::registration::RegistrationResult();
        template_pcd_info.cd_distance = 0.;
        //相似车型都检测
        if (carType == "MILA-L2H1" || carType == "MILA-L2H2")
        {
            if (template_pcd_info.car_type == "MILA-L2H1" || template_pcd_info.car_type == "MILA-L2H2")
            {
                IsCarType = true;
                std::cout << "Matching with " << template_pcd_info.car_type << "..." << std::endl;

                auto pcd_online_down = pcd_online->VoxelDownSample(_templates[0].voxel_size);
                //====================================Matching ICP========================================//
                LOG_INFO("ICP: Matching with {}...", template_pcd_info.car_type);
                auto t1 = std::chrono::high_resolution_clock::now();
                auto pcd_online_down_croped = pcd_online_down->Crop(open3d::geometry::AxisAlignedBoundingBox(
                    template_pcd_info.min_bound, template_pcd_info.max_bound));

                // ICP
                PointCloudRegistrationOpen3D::PreprocessOptions source_optins;
                source_optins.downsample_flag = false;  // already downsampled when init
                source_optins.voxel_size = template_pcd_info.voxel_size;
                source_optins.filter_flag = false;
                source_optins.estimate_normals_flag = false;


                PointCloudRegistrationOpen3D::PreprocessOptions target_options;
                target_options.downsample_flag = false;  // already downsampled when init
                target_options.voxel_size = template_pcd_info.voxel_size;
                target_options.filter_flag = false;
                target_options.estimate_normals_flag = false;

                PointCloudRegistrationOpen3D::RegistrationOptions reg_options;
                reg_options.voxel_size = template_pcd_info.voxel_size;
                reg_options.icp_distance_th = template_pcd_info.voxel_size + 2;//配准阈值，在设定的阈值内寻找配准点云

                Eigen::Matrix4d T_target_source = Eigen::Matrix4d::Identity();
                PointCloudRegistrationOpen3D reg;
                auto reg_result = reg.PCDRegistration(pcd_online_down_croped, source_optins, template_pcd_info.pcd_cropped_ptr, target_options,
                    reg_options, T_target_source, false);

                template_pcd_info.reg_result = reg_result;
                auto t2 = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() * 0.001;
                LOG_DEBUG("ICP Calculation Time: {} seconds", duration);

                //====================================calculate disAB========================================//
                LOG_DEBUG("CD: Matching with {}...", template_pcd_info.car_type);
                auto t3 = std::chrono::high_resolution_clock::now();
                double distAB = 0.0;
#pragma omp parallel for
                for (const auto& point : pcd_online_down->points_)
                {
                    std::vector<int> indices(1);//point索引
                    std::vector<double> dists(1);//distance
                    if (template_pcd_info.kdtree_ptr->SearchKNN(point, 1, indices, dists) > 0) {
                        distAB += std::sqrt(dists[0]);
                    }
                }
                template_pcd_info.cd_distance = distAB / pcd_online_down->points_.size();
                auto t4 = std::chrono::high_resolution_clock::now();
                auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() * 0.001;
                LOG_DEBUG("CD Calculation Time: {} seconds", duration2);

                continue;
            }
        }
        //相似车型都检测
        if (carType == "MILA-L1H1" || carType == "MILA-L1H2")
        {
            if (template_pcd_info.car_type == "MILA-L1H1" || template_pcd_info.car_type == "MILA-L1H2")
            {
                IsCarType = true;
                std::cout << "Matching with " << template_pcd_info.car_type << "..." << std::endl;

                auto pcd_online_down = pcd_online->VoxelDownSample(_templates[0].voxel_size);
                //====================================Matching ICP========================================//
                LOG_INFO("ICP: Matching with {}...", template_pcd_info.car_type);
                auto t1 = std::chrono::high_resolution_clock::now();
                auto pcd_online_down_croped = pcd_online_down->Crop(open3d::geometry::AxisAlignedBoundingBox(
                    template_pcd_info.min_bound, template_pcd_info.max_bound));

                // ICP
                PointCloudRegistrationOpen3D::PreprocessOptions source_optins;
                source_optins.downsample_flag = false;  // already downsampled when init
                source_optins.voxel_size = template_pcd_info.voxel_size;
                source_optins.filter_flag = false;
                source_optins.estimate_normals_flag = false;


                PointCloudRegistrationOpen3D::PreprocessOptions target_options;
                target_options.downsample_flag = false;  // already downsampled when init
                target_options.voxel_size = template_pcd_info.voxel_size;
                target_options.filter_flag = false;
                target_options.estimate_normals_flag = false;

                PointCloudRegistrationOpen3D::RegistrationOptions reg_options;
                reg_options.voxel_size = template_pcd_info.voxel_size;
                reg_options.icp_distance_th = template_pcd_info.voxel_size + 2;//配准阈值，在设定的阈值内寻找配准点云

                Eigen::Matrix4d T_target_source = Eigen::Matrix4d::Identity();
                PointCloudRegistrationOpen3D reg;
                auto reg_result = reg.PCDRegistration(pcd_online_down_croped, source_optins, template_pcd_info.pcd_cropped_ptr, target_options,
                    reg_options, T_target_source, false);

                template_pcd_info.reg_result = reg_result;
                auto t2 = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() * 0.001;
                LOG_DEBUG("ICP Calculation Time: {} seconds", duration);

                //====================================calculate disAB========================================//
                LOG_DEBUG("CD: Matching with {}...", template_pcd_info.car_type);
                auto t3 = std::chrono::high_resolution_clock::now();
                double distAB = 0.0;
#pragma omp parallel for
                for (const auto& point : pcd_online_down->points_)
                {
                    std::vector<int> indices(1);//point索引
                    std::vector<double> dists(1);//distance
                    if (template_pcd_info.kdtree_ptr->SearchKNN(point, 1, indices, dists) > 0) {
                        distAB += std::sqrt(dists[0]);
                    }
                }
                template_pcd_info.cd_distance = distAB / pcd_online_down->points_.size();
                auto t4 = std::chrono::high_resolution_clock::now();
                auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() * 0.001;
                LOG_DEBUG("CD Calculation Time: {} seconds", duration2);

                continue;
            }
        }
        //除相似车型以外的车辆 只检测一次
        if (template_pcd_info.car_type == carType)
        {
            IsCarType = true;
            std::cout << "Matching with " << template_pcd_info.car_type << "..." << std::endl;

            auto pcd_online_down = pcd_online->VoxelDownSample(_templates[0].voxel_size);
            //====================================Matching ICP========================================//
            LOG_INFO("ICP: Matching with {}...", template_pcd_info.car_type);
            auto t1 = std::chrono::high_resolution_clock::now();
            auto pcd_online_down_croped = pcd_online_down->Crop(open3d::geometry::AxisAlignedBoundingBox(
                template_pcd_info.min_bound, template_pcd_info.max_bound));

            // ICP
            PointCloudRegistrationOpen3D::PreprocessOptions source_optins;
            source_optins.downsample_flag = false;  // already downsampled when init
            source_optins.voxel_size = template_pcd_info.voxel_size;
            source_optins.filter_flag = false;
            source_optins.estimate_normals_flag = false;


            PointCloudRegistrationOpen3D::PreprocessOptions target_options;
            target_options.downsample_flag = false;  // already downsampled when init
            target_options.voxel_size = template_pcd_info.voxel_size;
            target_options.filter_flag = false;
            target_options.estimate_normals_flag = false;

            PointCloudRegistrationOpen3D::RegistrationOptions reg_options;
            reg_options.voxel_size = template_pcd_info.voxel_size;
            reg_options.icp_distance_th = template_pcd_info.voxel_size + 2;//配准阈值，在设定的阈值内寻找配准点云

            Eigen::Matrix4d T_target_source = Eigen::Matrix4d::Identity();
            PointCloudRegistrationOpen3D reg;
            auto reg_result = reg.PCDRegistration(pcd_online_down_croped, source_optins, template_pcd_info.pcd_cropped_ptr, target_options,
                reg_options, T_target_source, false);

            template_pcd_info.reg_result = reg_result;
            auto t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() * 0.001;
            LOG_DEBUG("ICP Calculation Time: {} seconds", duration);

            //====================================calculate disAB========================================//
            LOG_DEBUG("CD: Matching with {}...", template_pcd_info.car_type);
            auto t3 = std::chrono::high_resolution_clock::now();
            double distAB = 0.0;
#pragma omp parallel for
            for (const auto& point : pcd_online_down->points_)
            {
                std::vector<int> indices(1);//point索引
                std::vector<double> dists(1);//distance
                if (template_pcd_info.kdtree_ptr->SearchKNN(point, 1, indices, dists) > 0) 
                {
                    distAB += std::sqrt(dists[0]);
                }
            }
            template_pcd_info.cd_distance = distAB / pcd_online_down->points_.size();
            auto t4 = std::chrono::high_resolution_clock::now();
            auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() * 0.001;
            LOG_DEBUG("CD Calculation Time: {} seconds", duration2);

            break;
        }
    }
    if (IsCarType)
    {
        // 排序
        std::sort(_templates.begin(), _templates.end(), [](const PointCloudInfo& a, const PointCloudInfo& b) {
            return a.reg_result.fitness_ > b.reg_result.fitness_;
            });

        std::stringstream ss;
        for (const auto& template_pcd_info : _templates)
        {
            ss << "\nMatch Result with " << template_pcd_info.car_type << ":\n"
                << "  Fitness:\t" << template_pcd_info.reg_result.fitness_ << std::endl
                << "  CD Distance:\t" << template_pcd_info.cd_distance << std::endl;
        }
        LOG_DEBUG(ss.str());
        if ((_templates[0].reg_result.fitness_ > 0.9) &&(_templates[0].cd_distance < 50))
        {
            SpeedLogger::getInstance()->unInit();
            return _templates;//_templates[0]
        }
    }
    std::vector<PointCloudInfo>Unknown(1);
    Unknown[0].reg_result.fitness_ = 0.0;
    Unknown[0].cd_distance = 0.0;
    Unknown[0].car_type = "UnKnown";
    LOG_DEBUG("No carType: {}", carType);
    SpeedLogger::getInstance()->unInit();
    return  Unknown;
}
