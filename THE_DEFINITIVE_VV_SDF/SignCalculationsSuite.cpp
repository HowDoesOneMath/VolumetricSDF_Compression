#include "SignCalculationsSuite.h"

void SignCalculationsSuite::LoadSequences()
{
	sf.FindFiles(sequence_folder, mesh_sf);
}

void SignCalculationsSuite::InitializeSDF()
{
	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / (grid_width_voxels - 1);

	sdf.InitializeGrid(gds);
}

void SignCalculationsSuite::ReadAndCleanMesh(std::string to_read, VV_Mesh* mesh, double artifact_size)
{
	mesh->ReadOBJ(to_read);

	mp.CreateUnionFindPartitions(*mesh);
	mp.NegateInsignificantPartitions(*mesh, artifact_size);
	mesh->ClearNegativeTriangles();
	mp.ClearPartitions();
}

void SignCalculationsSuite::run(int argc, char** argv)
{
	LoadSequences();
	InitializeSDF();

	auto file_vector = sf.GetFileVector(mesh_sf.key);

	std::vector<unsigned char> compressed_signs;
	std::vector<uint8_t> lz_output;

	size_t total_sign_size = 0;
	size_t total_grid_size = 0;

	for (size_t i = 0; i < file_vector->size(); ++i)
	{
		std::cout << "Loading mesh " << i << " ... " << std::endl;

		ReadAndCleanMesh((*file_vector)[i], &input_mesh, artifact_size);
		sdf.CastMeshUnsignedDistance(&input_mesh, sdf_buffer_distance);

		sdf.HarvestSigns(compressed_signs);

		if (!lze.CompressData(compressed_signs.data(), compressed_signs.size(), lz_output))
		{
			std::cout << "Problem with LZ compression of signs!" << std::endl;
		}
		
		total_sign_size += (lz_output.size() + sizeof(size_t));

		if (!lze.CompressData(reinterpret_cast<unsigned char*>(sdf.GetQuantizedGridPointer()),sdf.GetQuantizedGridLength(), lz_output))
		{
			std::cout << "Problem with LZ compression of signs!" << std::endl;
		}

		total_grid_size += (lz_output.size() + sizeof(size_t));
	}

	std::cout << "Memory required to store signs: " << total_sign_size << std::endl;
	std::cout << "Memory required to store grid: " << total_grid_size << std::endl;
}
