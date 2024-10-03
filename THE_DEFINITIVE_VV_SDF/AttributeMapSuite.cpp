#include "AttributeMapSuite.h"

void AttributeMapSuite::RegenerateAttributeMapVSMC()
{
	test_mesh.ReadOBJ(input_mesh);

	//auto new_mesh = test_mesh.DecimateEdges(decimation_ratio);

	auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(test_mesh.vertices);
	auto cgal_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, test_mesh.vertices);
	vcm.CleanMeshCGAL(*cgal_mesh);
	auto cgal_aabb_tree = vcm.CreateAABB(*cgal_mesh);

	auto cgal_decimated = vcm.DecimateCGAL_Mesh(*cgal_mesh, decimation_ratio);
	std::shared_ptr<VV_Mesh> new_mesh = std::make_shared<VV_Mesh>();
	vcm.CopyCGAL_To_VV_Attribute(*cgal_decimated, new_mesh->vertices);
	
	new_mesh->GenerateNewUVsWithUVAtlas(attribute_map_size, attribute_map_size, gutter);
	new_mesh->SubdivideMesh(subdivision_count);
	
	//auto displaced_mesh = vcm.GetDisplacedMesh(*new_mesh, test_mesh, *cgal_mesh, *cgal_remap, *cgal_aabb_tree);
	auto displacement_vectors = vcm.GetMeshDisplacements(*new_mesh, test_mesh, *cgal_mesh, *cgal_remap, *cgal_aabb_tree);

	for (size_t i = 0; i < displacement_vectors->size(); ++i)
	{
		new_mesh->vertices.elements[i] += (*displacement_vectors)[i];
	}

	original_image.assign(input_texture.c_str());

	reconstructed_image.assign(attribute_map_size, attribute_map_size, 1, 3);
	reconstructed_image.fill(0);


	std::cout << "Remapping..." << std::endl;

	//tr.Remap(test_mesh, *new_mesh, original_image, reconstructed_image);

	auto remap_begin = std::chrono::high_resolution_clock::now();

	//tr.Remap(test_mesh, *displaced_mesh, original_image, reconstructed_image, uv_remap_epsilon);
	tr.Remap(test_mesh, *new_mesh, original_image, reconstructed_image, uv_remap_epsilon);

	auto remap_end = std::chrono::high_resolution_clock::now();

	std::cout << "Time taken to remap: " << ((remap_end - remap_begin).count() * 0.000000001) << std::endl;


	//displaced_mesh->WriteOBJ(output_mesh);
	new_mesh->WriteOBJ(output_mesh_vsmc);

	reconstructed_image.save_png(output_texture_vsmc.c_str());
}

void AttributeMapSuite::RegenerateAttributeMapSDF()
{
	test_mesh.ReadOBJ(input_mesh);
	original_image.assign(input_texture.c_str());

	mp.CreateUnionFindPartitions(test_mesh);
	mp.NegateInsignificantPartitions(test_mesh, artifact_size);
	test_mesh.ClearNegativeTriangles();
	mp.ClearPartitions();

	reconstructed_image.assign(attribute_map_size, attribute_map_size, 1, 3);
	reconstructed_image.fill(0);

	GridDataStruct gds(
		grid_width_voxels, 2 * grid_width_voxels, grid_width_voxels,
		grid_center.x(), grid_center.y(), grid_center.z(),
		grid_width_meters / (grid_width_voxels - 1)
	);

	if (!sdf.InitializeGrid(gds))
	{
		return;
	}

	sdf.CastMeshUnsignedDistance(&test_mesh, sdf_buffer_distance);

	Eigen::Vector2i partitions_vec2 = Eigen::Vector2i(estimated_uv_partitions, estimated_uv_partitions);
	auto new_mesh = sdf.ExtractMeshQuantized();// partitions_vec2, uv_partition_buffer);

	auto remap_begin = std::chrono::high_resolution_clock::now();

	if (!tr.RemapWithPartitions(test_mesh, *new_mesh, original_image, reconstructed_image, partitions_vec2, uv_partition_buffer, padding_loops))
	{
		return;
	}

	auto remap_end = std::chrono::high_resolution_clock::now();

	std::cout << "Time taken to remap: " << ((remap_end - remap_begin).count() * 0.000000001) << std::endl;

	new_mesh->WriteOBJ(output_mesh_sdf);

	reconstructed_image.save_png(output_texture_sdf.c_str());
}

void AttributeMapSuite::SingleTestDecimation()
{
	test_mesh.ReadOBJ(input_mesh);

	std::cout << "Decimating..." << std::endl;

	//auto new_mesh = test_mesh.DecimateEdges(decimation_ratio);

	auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(test_mesh.vertices);
	auto cgal_decimated = vcm.DecimateCGAL_Mesh(*cgal_mesh, decimation_ratio);
	std::shared_ptr<VV_Mesh> new_mesh = std::make_shared<VV_Mesh>();
	vcm.CopyCGAL_To_VV_Attribute(*cgal_decimated, new_mesh->vertices);
}

void AttributeMapSuite::ReadJPGTest()
{

}

void AttributeMapSuite::run(int argc, char** argv)
{
	//RegenerateAttributeMapVSMC();
	RegenerateAttributeMapSDF();

	//SingleTestDecimation();
}
