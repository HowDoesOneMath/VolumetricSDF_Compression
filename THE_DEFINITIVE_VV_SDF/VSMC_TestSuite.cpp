#include "VSMC_TestSuite.h"

void VSMC_TestSuite::CompressSingularMesh()
{
	VV_Mesh test_mesh;
	VV_Mesh output_mesh;
	cimg_library::CImg<unsigned char> test_texture;
	cimg_library::CImg<unsigned char> output_texture;

	test_mesh.ReadOBJ(test_mesh_name);
	test_texture.assign(test_texture_name.c_str());
	output_texture.assign(atlas_size, atlas_size, 1, 3);

	auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(test_mesh.vertices);
	auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, test_mesh.vertices);
	vcm.CleanMeshCGAL(*cgal_mesh);
	auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

	auto decim_mesh = vcm.DecimateCGAL_Mesh(*cgal_mesh, decimation_ratio);

	if (!vcm.CopyCGAL_To_VV_Attribute(*decim_mesh, output_mesh.vertices))
	{
		std::cout << "ERROR - could not copy CGAL attribute!" << std::endl;
		return;
	}

	std::cout << output_mesh.vertices.elements.size() << ", " << output_mesh.vertices.indices.size() << std::endl;

	if (!output_mesh.GenerateNewUVsWithUVAtlas(atlas_size, atlas_size, gutter_size))
	{
		std::cout << "ERROR - could not generate UVs!" << std::endl;
		return;
	}

	output_mesh.WriteOBJ(output_base_mesh_name.c_str());

	if (!tr.Remap(test_mesh, output_mesh, test_texture, output_texture, uv_epsilon))
	{
		std::cout << "ERROR - could not remap textures!" << std::endl;
		return;
	}

	auto vertex_adjacencies = output_mesh.SubdivideMeshAndGetAdjacencies(subdiv_loops);

	auto displacements = vcm.GetMeshDisplacements(output_mesh, test_mesh, *cgal_mesh, *index_remap, *aabb_tree);

	cimg_library::CImg<unsigned char> displacement_image(displacement_texture_size, displacement_texture_size, 1, 3, 0);

	Eigen::Vector3d max_data_val = Eigen::Vector3d::Ones() * -DBL_MAX;
	Eigen::Vector3d min_data_val = Eigen::Vector3d::Ones() * DBL_MAX;

	for (size_t i = 0; i < displacements->size(); ++i)
	{
		min_data_val = min_data_val.cwiseMin((*displacements)[i]);
		max_data_val = max_data_val.cwiseMax((*displacements)[i]);
	}

	FillImageBlocksRaster(displacement_image, displacement_block_size, *displacements, (unsigned char)0, (unsigned char)255, min_data_val, max_data_val);

	output_mesh.ConcatenateMesh(test_mesh, Eigen::Vector3d(0.7, 0, 0));

	output_mesh.WriteOBJ(output_displaced_mesh_name.c_str());
	output_texture.save_png(output_texture_name.c_str());
	displacement_image.save_png(output_displacement_name.c_str());
}

void VSMC_TestSuite::TestGetAdjacencies()
{
	VV_Mesh test_mesh;
	VV_Mesh output_mesh;

	test_mesh.ReadOBJ(test_mesh_name);

	auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(test_mesh.vertices);
	auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, test_mesh.vertices);
	vcm.CleanMeshCGAL(*cgal_mesh);
	auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

	auto decim_mesh = vcm.DecimateCGAL_Mesh(*cgal_mesh, decimation_ratio);

	if (!vcm.CopyCGAL_To_VV_Attribute(*decim_mesh, output_mesh.vertices))
	{
		std::cout << "ERROR - could not copy CGAL attribute!" << std::endl;
		return;
	}

	std::cout << output_mesh.vertices.elements.size() << ", " << output_mesh.vertices.indices.size() << std::endl;

	if (!output_mesh.GenerateNewUVsWithUVAtlas(atlas_size, atlas_size, gutter_size))
	{
		std::cout << "ERROR - could not generate UVs!" << std::endl;
		return;
	}

	auto vertex_adjacencies = output_mesh.SubdivideMeshAndGetAdjacencies(subdiv_loops);

	auto displacements = vcm.GetMeshDisplacements(output_mesh, test_mesh, *cgal_mesh, *index_remap, *aabb_tree);

	for (size_t layer = 0; layer < vertex_adjacencies->size(); ++layer)
	{
		std::cout << "Current layer: " << layer << " (" << (*vertex_adjacencies)[layer].first << " elements)" << std::endl;
	
		for (size_t i = 0; i < (*vertex_adjacencies)[layer].second.size(); ++i)
		{
			auto current_tuple = (*vertex_adjacencies)[layer].second[i];
	
			std::cout << "\t" << i << " (" << std::get<0>(current_tuple) << ", " << std::get<1>(current_tuple) <<
				") : " << std::get<2>(current_tuple).size() << std::endl;
		}
	}
}

void VSMC_TestSuite::CompressSequence(std::string root_folder, std::string save_name)
{
	std::filesystem::create_directories(compressed_sequence_folder);
	std::filesystem::create_directories(reconstructed_sequence_folder);

	vsmc_comp.InitializeCompressor(decimation_ratio, atlas_size, atlas_size, gutter_size, subdiv_loops, 
		displacement_texture_size, displacement_block_size, push_pull_kernel_size, push_pull_kernel_scale, 
		draco_compression_level);

#if VSMC_TIME_LOGGING
	vsmc_comp.SetTimeLogFile(time_log_path);
#endif

	//if (!vsmc_comp.CompressSequence(root_folder, mesh_sf, texture_sf, save_name, texture_tag, displacement_tag))
	if (!vsmc_comp.CompressSequence(root_folder, mesh_sf, texture_sf, save_name, texture_tag, displacement_tag, beginning_frame, end_frame))
	{
		std::cout << "Compression failed!" << std::endl;
		return;
	}

	if (!vsmc_comp.DecompressSequence(save_name, compressed_sequence_folder, displacement_sf, reconstructed_mesh_tag))
	{
		std::cout << "Decompression failed!" << std::endl;
		return;
	}
}

void VSMC_TestSuite::run(int argc, char** argv)
{
	//TestGetAdjacencies();

	//CompressSingularMesh();

	CompressSequence(test_sequence, compress_file_output);
}
