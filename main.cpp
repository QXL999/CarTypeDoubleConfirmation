#include "CarTypeDoubleConfirmation.h"	
#include <chrono>
#include <filesystem>

namespace fs = std::filesystem;

int main() 
{
	// init algorithm
	// 初始化一次就好
	std::string templates_dir = R"(..\Templates)";
	CarTypeDoubleConfirmation::getInstance()->init(templates_dir);



	// load online ply_ptr
	std::string data_dir = R"(..\All)";
	//load cartype
	std::string carType = "SV51-01";
	for (const auto& entry : fs::directory_iterator(data_dir)) 
	{
		if (entry.is_regular_file()) 
		{
			if (entry.path().extension() == ".ply") 
			{
				std::string online_pcd_path = entry.path().string();
				std::cout << "Testing " << online_pcd_path << std::endl;
				auto online_pcd_ptr = std::make_shared<open3d::geometry::PointCloud>();
				open3d::io::ReadPointCloud(online_pcd_path, *online_pcd_ptr);
				if (online_pcd_ptr->IsEmpty())
				{

					std::cout << "Error: Failed to read online ply file -> " << online_pcd_path << std::endl;
					return -1;
				}
				auto t1 = std::chrono::high_resolution_clock::now();
				auto match_results = CarTypeDoubleConfirmation::getInstance()->calcMatchResultsMultiThreads(online_pcd_ptr, carType);

				auto t2 = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
				std::cout << "Algo Time: " << duration * 0.001 << " s." << std::endl;
				//返回在线点云与所有模板的配准结果
				for (auto match_result : match_results)
				{
					std::cout << "Match Result with " << match_result.car_type << ":\n"
						<< "  Fitness:\t" << match_result.reg_result.fitness_ << std::endl
						<< "  CD Distance:\t" << match_result.cd_distance << std::endl;
					std::cout << "-------------------------" << std::endl;
				}


				std::string output_dir = R"(..\TestResult)";
				//算法内部做好排序，若配准成功，match_results[0]即为正确配准车型

				//增加阈值判断标准
				std::string output_res = match_results[0].car_type;
				std::cout <<"Result: " << output_res << std::endl;
				//open3d::io::WritePointCloud(output_path, *online_pcd_ptr);
			}
		}
	}

	system("pause");
	return 0;
}
