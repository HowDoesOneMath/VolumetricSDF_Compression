#include "ErrorMetricSuite.h"

void ErrorMetricSuite::RunAllSets()
{
	for (size_t i = 0; i < em_set.size(); ++i)
	{
		to_write.open(em_set[i].file_name);	
		comparator_seq.files.clear();

		if (!comparator_seq.FindFiles(em_set[i].original_sequence, mesh_sf_original))
		{
			std::cout << "Error reading original sequence " << em_set[i].original_sequence << "!" << std::endl;
			to_write.close();
			continue;
		}

		auto min_max_bb = GetBoundingBox(comparator_seq.files[mesh_sf_original.key]);

		double diag_dist = (min_max_bb.first - min_max_bb.second).norm();

		size_t file_count = comparator_seq.files[mesh_sf_original.key].size();

		for (size_t j = 0; j < em_set[i].to_compare.size(); ++j)
		{
			comparator_seq.files[mesh_sf_to_compare.key].clear();
			std::cout << "Processing " << em_set[i].original_sequence << " -> " << em_set[i].to_compare[j] << "... " << std::endl;

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

			VV_Mesh original_mesh;
			VV_Mesh reconstructed_mesh;

			for (size_t k = 0; k < file_count; ++k)
			{
				original_mesh.ReadOBJ(comparator_seq.files[mesh_sf_original.key][k]);
				auto pc_original = pcg.GeneratePointsFromMesh(original_mesh, diag_dist * point_cloud_spacing, epsilon_vector);
				reconstructed_mesh.ReadOBJ(comparator_seq.files[mesh_sf_to_compare.key][k]);
				auto pc_reconstructed = pcg.GeneratePointsFromMesh(reconstructed_mesh, diag_dist * point_cloud_spacing, epsilon_vector);

				auto hc_pair = mem.GetHausdorffChamferDistanceMetric(original_mesh, reconstructed_mesh, *pc_original, *pc_reconstructed);
				auto point2point = mem.GetPointToPointMetric(*pc_original, *pc_reconstructed);
				auto point2plane = mem.GetPointToPlaneMetric(*pc_original, *pc_reconstructed);

				greatest_hausdorff = std::max(greatest_hausdorff, hc_pair.first);
				greatest_chamfer = std::max(greatest_chamfer, hc_pair.second);
				greatest_point2point = std::max(greatest_point2point, point2point.first);
				greatest_point2plane = std::max(greatest_point2plane, point2plane.first);

				total_hausdorff += hc_pair.first;
				total_chamfer += hc_pair.second;
				total_point2point += point2point.first;
				total_point2plane += point2plane.first;
			}


			to_write << "\tHAUSDORFF\n\t\tAvg: " << (total_hausdorff / file_count) << "\n\t\tMax: " << greatest_hausdorff << "\n";
			to_write << "\tCHAMFER\n\t\tAvg: " << (total_chamfer / file_count) << "\n\t\tMax: " << greatest_chamfer << "\n";
			to_write << "\tPOINT TO POINT\n\t\tAvg: " << (total_point2point / file_count) << "\n\t\tMax: " << greatest_point2point << "\n";
			to_write << "\tPOINT TO PLANE\n\t\tAvg: " << (total_point2plane / file_count) << "\n\t\tMax: " << greatest_point2plane << "\n";

			to_write << "\n";

		}


		to_write.close();
	}
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> ErrorMetricSuite::GetBoundingBox(std::vector<std::string>& input_meshes)
{
	auto to_return = std::make_pair<Eigen::Vector3d, Eigen::Vector3d>(
		Eigen::Vector3d(DBL_MAX, DBL_MAX, DBL_MAX),
		Eigen::Vector3d(-DBL_MAX, -DBL_MAX, -DBL_MAX)
	);

	VV_Mesh mesh_to_find_bb_of;

	for (size_t i = 0; i < input_meshes.size(); ++i)
	{
		mesh_to_find_bb_of.ReadOBJ(input_meshes[i]);
		
		auto m_bb = mesh_to_find_bb_of.GetBoundingBox();

		to_return.first = to_return.first.cwiseMin(m_bb.first);
		to_return.second = to_return.second.cwiseMax(m_bb.second);
	}

	return to_return;
}

void ErrorMetricSuite::run(int argc, char** argv)
{

}
