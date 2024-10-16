#include "ErrorMetricSuite.h"

void ErrorMetricSuite::RunAllSets()
{
	std::cout << "Sets: " << em_set.size() << std::endl;

	for (size_t i = 0; i < em_set.size(); ++i)
	{
		std::cout << "Set " << em_set[i].original_sequence << "..." << std::endl;

		to_write.open(em_set[i].file_name);	
		comparator_seq.files.clear();

		if (!comparator_seq.FindFiles(em_set[i].original_sequence, mesh_sf_original))
		{
			std::cout << "Error reading original sequence " << em_set[i].original_sequence << "!" << std::endl;
			to_write.close();
			continue;
		}

		std::cout << "Creating BB..." << std::endl;

		auto tp_bb = std::chrono::high_resolution_clock::now();
		auto min_max_bb = GetBoundingBox(comparator_seq.files[mesh_sf_original.key]);
		auto delta_bb = std::chrono::high_resolution_clock::now() - tp_bb;

		double diag_dist = (min_max_bb.first - min_max_bb.second).norm();

		size_t file_count = comparator_seq.files[mesh_sf_original.key].size();

		std::cout << "BB size: " << min_max_bb.first.transpose() << " --- " << min_max_bb.second.transpose() << 
			" (" << (min_max_bb.second - min_max_bb.first).transpose() << ")" << std::endl;
		std::cout << "\tFinished in " << delta_bb.count() * 0.000000001 << " seconds" << std::endl;

		for (size_t j = 0; j < em_set[i].to_compare.size(); ++j)
		{
			comparator_seq.files[mesh_sf_to_compare.key].clear();
			std::cout << "Processing " << em_set[i].original_sequence << " -> " << em_set[i].to_compare[j] << "... " << std::endl;

			auto tp_comp = std::chrono::high_resolution_clock::now();

			if (!comparator_seq.FindFiles(em_set[i].to_compare[j], mesh_sf_to_compare))
			{
				std::cout << "Error reading comparison sequence " << em_set[i].to_compare[j] << "!" << std::endl;
				continue;
			}

			if (comparator_seq.files[mesh_sf_to_compare.key].size() != comparator_seq.files[mesh_sf_original.key].size())
			{
				std::cout << "Invalid file numbers! ("
					<< em_set[i].original_sequence << " --- " << comparator_seq.files[mesh_sf_original.key].size() << ", "
					<< em_set[i].to_compare[j] << " --- " << comparator_seq.files[mesh_sf_to_compare.key].size() << ")" << std::endl;
				continue;
			}

			to_write << em_set[i].to_compare[j] << ": \n";

			double greatest_hausdorff = -DBL_MAX;
			double greatest_chamfer = -DBL_MAX;
			double greatest_point2point = -DBL_MAX;
			double greatest_point2plane = -DBL_MAX;

			double total_hausdorff = 0;
			double total_chamfer = 0;
			double total_point2point = 0;
			double total_point2plane = 0;

			std::vector<double> hausdorff_distances;
			std::vector<double> chamfer_distances;
			std::vector<double> point2point_distances;
			std::vector<double> point2plane_distances;

			hausdorff_distances.resize(file_count);
			chamfer_distances.resize(file_count);
			point2point_distances.resize(file_count);
			point2plane_distances.resize(file_count);

#pragma omp parallel for
			for (int k = 0; k < file_count; ++k)
			{
				VV_Mesh original_mesh;
				VV_Mesh reconstructed_mesh;

				MeshEvaluationMetrics mem;

				original_mesh.ReadOBJ(comparator_seq.files[mesh_sf_original.key][k]);
				auto pc_original = pcg.GeneratePointsFromMesh(original_mesh, diag_dist * point_cloud_spacing, epsilon_vector);
				reconstructed_mesh.ReadOBJ(comparator_seq.files[mesh_sf_to_compare.key][k]);
				auto pc_reconstructed = pcg.GeneratePointsFromMesh(reconstructed_mesh, diag_dist * point_cloud_spacing, epsilon_vector);

				auto hc_pair = mem.GetHausdorffChamferDistanceMetric(original_mesh, reconstructed_mesh, *pc_original, *pc_reconstructed);
				auto point2point = mem.GetPointToPointMetric(*pc_original, *pc_reconstructed);
				auto point2plane = mem.GetPointToPlaneMetric(*pc_original, *pc_reconstructed);

				hausdorff_distances[k] = hc_pair.first;
				chamfer_distances[k] = hc_pair.second;
				point2point_distances[k] = point2point.second;
				point2plane_distances[k] = point2plane.second;
			}

			for (size_t k = 0; k < file_count; ++k)
			{
				greatest_hausdorff = std::max(greatest_hausdorff, hausdorff_distances[k]);
				greatest_chamfer = std::max(greatest_chamfer, chamfer_distances[k]);
				greatest_point2point = std::max(greatest_point2point, point2point_distances[k]);
				greatest_point2plane = std::max(greatest_point2plane, point2plane_distances[k]);

				total_hausdorff += hausdorff_distances[k];
				total_chamfer += chamfer_distances[k];
				total_point2point += point2point_distances[k];
				total_point2plane += point2plane_distances[k];
			}

			auto delta_comp = std::chrono::high_resolution_clock::now() - tp_comp;

			std::cout << "\tFinished in " << delta_comp.count() * 0.000000001 << " seconds" << std::endl;

			to_write << "\tHAUSDORFF\n\t\tAvg: " << (total_hausdorff / file_count) << "\n\t\tMax: " << greatest_hausdorff << "\n";
			to_write << "\tCHAMFER\n\t\tAvg: " << (total_chamfer / file_count) << "\n\t\tMax: " << greatest_chamfer << "\n";
			to_write << "\tPOINT TO POINT\n\t\tAvg: " << (total_point2point / file_count) << "\n\t\tMax: " << greatest_point2point << "\n";
			to_write << "\tPOINT TO PLANE\n\t\tAvg: " << (total_point2plane / file_count) << "\n\t\tMax: " << greatest_point2plane << "\n";

			to_write << "\n";

		}

		std::cout << "\n" << std::endl;

		to_write.close();
	}
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> ErrorMetricSuite::GetBoundingBox(std::vector<std::string>& input_meshes)
{
	auto to_return = std::make_pair<Eigen::Vector3d, Eigen::Vector3d>(
		Eigen::Vector3d(DBL_MAX, DBL_MAX, DBL_MAX),
		Eigen::Vector3d(-DBL_MAX, -DBL_MAX, -DBL_MAX)
	);

	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> bbs;
	bbs.resize(input_meshes.size());

#pragma omp parallel for
	for (int i = 0; i < input_meshes.size(); ++i)
	{
		VV_Mesh mesh_to_find_bb_of;

		mesh_to_find_bb_of.ReadOBJ(input_meshes[i]);
		
		bbs[i] = mesh_to_find_bb_of.GetBoundingBox();
	}

	for (size_t i = 0; i < bbs.size(); ++i)
	{
		to_return.first = to_return.first.cwiseMin(bbs[i].first);
		to_return.second = to_return.second.cwiseMax(bbs[i].second);
	}

	return to_return;
}

void ErrorMetricSuite::run(int argc, char** argv)
{
	RunAllSets();
}
