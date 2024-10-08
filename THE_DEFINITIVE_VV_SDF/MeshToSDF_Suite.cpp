#include "MeshToSDF_Suite.h"

void MeshToSDF_Suite::ConvertSequence()
{
	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	sdf.ClearGrid();
	sdf.InitializeGrid(gds);

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

		mesh.ClearUnreferencedElements();

		mp.CreateUnionFindPartitions(mesh);
		mp.NegateInsignificantPartitions(mesh, mesh_maximum_artifact_size);
		mesh.ClearNegativeTriangles();

		if (!sdf.CastMeshUnsignedDistance(&mesh, shell_size))
		{
			std::cout << "Problem creating shell of mesh " << std::to_string(i) << ": '" << sf.files[mesh_sf.key][i] << "'!" << std::endl;
			return;
		}

		auto shell_mesh = sdf.ExtractMeshQuantized();

		std::string save_name = output_mesh_tag + "_" + GetNumberFixedLength(i, digit_count) + ".obj";

		if (!shell_mesh->WriteOBJ(save_name))
		{
			std::cout << "Problem writing converted mesh " << save_name << " (" << sf.files[mesh_sf.key][i] << ")!" << std::endl;
			return;
		}
	}

	sdf.ClearGrid();
}

void MeshToSDF_Suite::run(int argc, char** argv)
{
	ConvertSequence();
}
