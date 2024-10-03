#include "UnsignedFieldSuite.h"

void UnsignedFieldSuite::TestOneMesh()
{

	if (!test_mesh.ReadOBJ(mesh_to_approximate))
	{
		std::cout << "Error when loading mesh!" << std::endl;
		return;
	}

	std::cout << "Initial Stats: " << test_mesh.vertices.indices.size() << ", " << test_mesh.vertices.elements.size() << std::endl;

	mp.CreateUnionFindPartitions(test_mesh);
	std::cout << "Bounding box: " << mp.absolute_upper_bound.transpose() << " --- " << mp.absolute_lower_bound.transpose() << std::endl;
	mp.NegateInsignificantPartitions(test_mesh, bad_mesh_threshold);
	test_mesh.ClearNegativeTriangles();
	mp.ClearPartitions();

	std::cout << "Post Artifact Removal Stats: " << test_mesh.vertices.indices.size() << ", " << test_mesh.vertices.elements.size() << std::endl;

	for (int i = 0; i < density_count; ++i)
	{
		GridDataStruct gds(
			grid_densities[i],
			2 * grid_densities[i],
			grid_densities[i],
			0, center, 0,
			grid_length / (grid_densities[i] - 1)
		);

		//Eigen::Vector2i estimated_uv_divisions = Eigen::Vector2i(2 * grid_densities[i], 2 * grid_densities[i]);
		if (!test_TSDF.InitializeGrid(gds))
		{
			std::cout << "Error when initialized grid!" << std::endl;
			return;
		}

		auto cast_begin = std::chrono::high_resolution_clock::now();

		auto expanded_mesh = test_TSDF.CastMeshUnsignedDistance(&test_mesh, buffer_distance * gds.unit_length);
		expanded_mesh->ClearNegativeTriangles();

		//test_TSDF.CastMeshUnsignedDistance(&test_mesh, buffer_distance * gds.unit_length);
		auto shrunk_mesh = test_TSDF.ExtractMeshQuantized();// estimated_uv_divisions, 0.05);

		auto cast_end = std::chrono::high_resolution_clock::now();

		std::cout << "Total cast time: " << ((cast_end - cast_begin).count() * 0.000000001) << std::endl;


		expanded_mesh->ConcatenateMesh(test_mesh, concatenate_offset);
		shrunk_mesh->ConcatenateMesh(test_mesh, concatenate_offset);

		expanded_mesh->WriteOBJ(output_mesh_without_tag + "_" + std::to_string(grid_densities[i]) + mesh_file_tag);
		shrunk_mesh->WriteOBJ(shrunk_output_mesh_without_tag + "_" + std::to_string(grid_densities[i]) + mesh_file_tag);

		std::cout << std::endl << std::endl;

		test_TSDF.ClearGrid();
	}

	test_mesh.Clear();
	mp.ClearPartitions();
}

void UnsignedFieldSuite::TestOneTriangle()
{
	for (int i = 0; i < density_count; ++i)
	{
		std::cout << i << "; SIZE: " << grid_densities[i] << std::endl;

		GridDataStruct gds(
			grid_densities[i],
			grid_densities[i],
			grid_densities[i],
			0, center, 0,
			grid_length / (grid_densities[i] - 1)
		);

		if (!test_TSDF.InitializeGrid(gds))
		{
			std::cout << "Error when initialized grid!" << std::endl;
			return;
		}

		test_mesh.Clear();

		test_mesh.vertices.elements.push_back(Eigen::Vector3d(-1.0, -1.0, -1.0));
		test_mesh.vertices.elements.push_back(Eigen::Vector3d(1.0, 1.0, -1.0));
		test_mesh.vertices.elements.push_back(Eigen::Vector3d(1.0, -1.0, 1.0));

		test_mesh.vertices.indices.push_back(Eigen::Vector3i(0, 1, 2));

		//auto expanded_mesh = test_TSDF.CastMeshUnsignedDistance(&test_mesh, buffer_distance * gds.unit_length);

		//expanded_mesh->WriteOBJ(single_triangle);

		std::cout << std::endl << std::endl;
	}
}

void UnsignedFieldSuite::run(int argc, char** argv)
{
	TestOneMesh();

	//TestOneTriangle();
}
