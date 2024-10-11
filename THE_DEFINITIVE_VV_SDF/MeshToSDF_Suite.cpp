#include "MeshToSDF_Suite.h"

void MeshToSDF_Suite::ConvertSequence()
{
	std::filesystem::create_directories(output_folder);

	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	sdf.ClearGrid();
	sdf.InitializeGrid(gds);

	Eigen::Vector3d bounding_box_max = -Eigen::Vector3d::Ones() * DBL_MAX;
	Eigen::Vector3d bounding_box_min = Eigen::Vector3d::Ones() * DBL_MAX;

	if (!sf.FindFiles(input_folder, mesh_sf))
	{
		std::cout << "Problem reading files!" << std::endl;
		return;
	}

	for (size_t i = 0; i < sf.files[mesh_sf.key].size(); ++i)
	{
		std::cout << "Converting mesh " << i << "..." << std::endl;

		if (!mesh.ReadOBJ(sf.files[mesh_sf.key][i]))
		{
			std::cout << "Problem reading mesh " << std::to_string(i) << ": '" << sf.files[mesh_sf.key][i] << "'!" << std::endl;
			return;
		}

		auto bb = mesh.GetBoundingBox();
		bounding_box_min = bounding_box_min.cwiseMin(bb.first);
		bounding_box_max = bounding_box_max.cwiseMax(bb.second);

		mesh.ClearUnreferencedElements();

		mp.CreateUnionFindPartitions(mesh);
		mp.NegateInsignificantPartitions(mesh, mesh_maximum_artifact_size);
		mesh.ClearNegativeTriangles();

		std::cout << "Tris: " << mesh.vertices.indices.size() << std::endl;

		if (!sdf.CastMeshUnsignedDistance(&mesh, shell_size))
		{
			std::cout << "Problem creating shell of mesh " << std::to_string(i) << ": '" << sf.files[mesh_sf.key][i] << "'!" << std::endl;
			return;
		}

		//sdf.PrintCrossSectionOfQuantizedGrid(64);

		auto shell_mesh = sdf.ExtractMeshQuantized();

		std::string save_name = output_mesh_tag + "_" + GetNumberFixedLength(i, digit_count) + ".obj";

		if (!shell_mesh->WriteOBJ(save_name))
		{
			std::cout << "Problem writing converted mesh " << save_name << " (" << sf.files[mesh_sf.key][i] << ")!" << std::endl;
			return;
		}
	}

	std::cout << "BOUNDING BOX OF SEQUENCE:\n\t" <<
		bounding_box_min.transpose() << "\n\t" <<
		bounding_box_max.transpose() << std::endl;

	std::cout << "SEQUENCE_DIMS:\n\t" << (bounding_box_max - bounding_box_min).transpose() << std::endl;
	std::cout << "SEQUENCE_CENTER:\n\t" << ((bounding_box_max + bounding_box_min) * 0.5).transpose() << std::endl;

	Eigen::Vector3d actual_bb_max = center + 0.5 * Eigen::Vector3d(gds.dim_x - 1, gds.dim_y - 1, gds.dim_z - 1) * gds.unit_length - Eigen::Vector3d::Ones() * shell_size;
	Eigen::Vector3d actual_bb_min = center - 0.5 * Eigen::Vector3d(gds.dim_x - 1, gds.dim_y - 1, gds.dim_z - 1) * gds.unit_length + Eigen::Vector3d::Ones() * shell_size;

	std::cout << "THIS_BOUNDING_BOX:\n\t" <<
		actual_bb_min.transpose() << "\n\t" <<
		actual_bb_max.transpose() << std::endl;

	std::cout << "THIS_BB_DIMS:\n\t" << (actual_bb_max - actual_bb_min).transpose() << std::endl;
	std::cout << "THIS_BB_CENTER:\n\t" << ((actual_bb_max + actual_bb_min) * 0.5).transpose() << std::endl;

	bool clears =
		(actual_bb_max.x() > bounding_box_max.x()) &&
		(actual_bb_max.y() > bounding_box_max.y()) &&
		(actual_bb_max.z() > bounding_box_max.z()) &&
		(actual_bb_min.x() < bounding_box_min.x()) &&
		(actual_bb_min.y() < bounding_box_min.y()) &&
		(actual_bb_min.z() < bounding_box_min.z());

	if (clears)
	{
		std::cout << "CLEAR!" << std::endl;
	}
	else
	{
		std::cout << "NOT CLEAR!" << std::endl;
	}

	sdf.ClearGrid();
}

void MeshToSDF_Suite::run(int argc, char** argv)
{
	ConvertSequence();
}
