#include "VV_SVD_TemporalSuite.h"

void VV_SVD_TemporalSuite::TestSingleSequence()
{
	std::filesystem::create_directories(intermediary_sequence_folder);
	std::filesystem::create_directories(compressed_sequence_folder);
	std::filesystem::create_directories(reconstructed_sequence_folder);

#if TSVD_TIME_LOGGING
	vv_svd_tc.SetTimeLogFile(time_log_path_final);
#endif

	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	std::cout << "Initializing..." << std::endl;

	if (!vv_svd_tc.Initialize(gds, block_size, max_frames_per_svd, patch_padding, island_padding, minimum_normal_similarity))
	{
		std::cout << "PROBLEM CREATING INTERMEDIARY FILE!" << std::endl;
		return;
	}

	std::cout << "Saving intermediary..." << std::endl;

	if (!vv_svd_tc.SaveIntermediaryFile(input_folder, intermediary_file, mesh_sf, shell_size, mesh_maximum_artifact_size))
	{
		std::cout << "PROBLEM CREATING INTERMEDIARY FILE!" << std::endl;
		return;
	}

	std::cout << "Saving final file..." << std::endl;

	if (!vv_svd_tc.SaveFinalFile(intermediary_file, final_file, significant_value_ratio, max_allowed_components))
	{
		std::cout << "PROBLEM CREATING FINAL FILE!" << std::endl;
		return;
	}

	std::cout << "Texturing meshes..." << std::endl;

	if (!vv_svd_tc.AugmentFinalFileWithTextureData(input_folder, mesh_sf, texture_sf, final_file, output_texture_tag, digit_count,
		Eigen::Vector2i(tex_size, tex_size), uv_epsilon, kernel_size, kernel_scale))
	{
		std::cout << "PROBLEM TEXTURING MESHES!" << std::endl;
		return;
	}
	std::cout << "Recreating meshes..." << std::endl;

	if (!vv_svd_tc.ReconstructMeshes(final_file, output_mesh_tag, digit_count))
	{
		std::cout << "PROBLEM RECONSTRUCTING MESHES!" << std::endl;
		return;
	}

#if TSVD_TIME_LOGGING
	vv_svd_tc.CloseTimeLogFile();
#endif
}

void VV_SVD_TemporalSuite::TestSingleIntermediary()
{
	std::filesystem::create_directories(intermediary_sequence_folder);

#if TSVD_TIME_LOGGING
	vv_svd_tc.SetTimeLogFile(time_log_path_intermediary);
#endif

	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	std::cout << "Initializing..." << std::endl;

	if (!vv_svd_tc.Initialize(gds, block_size, max_frames_per_svd, patch_padding, island_padding, minimum_normal_similarity))
	{
		std::cout << "PROBLEM CREATING INTERMEDIARY FILE!" << std::endl;
		return;
	}

	std::cout << "Saving intermediary..." << std::endl;

	if (!vv_svd_tc.SaveIntermediaryFile(input_folder, intermediary_file, mesh_sf, shell_size, mesh_maximum_artifact_size))
	{
		std::cout << "PROBLEM CREATING INTERMEDIARY FILE!" << std::endl;
		return;
	}

#if TSVD_TIME_LOGGING
	vv_svd_tc.CloseTimeLogFile();
#endif
}

void VV_SVD_TemporalSuite::TestFromPreExistingIntermediary()
{
	std::filesystem::create_directories(compressed_sequence_folder);
	std::filesystem::create_directories(reconstructed_sequence_folder);

#if TSVD_TIME_LOGGING
	vv_svd_tc.SetTimeLogFile(time_log_path_final);
#endif

	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	std::cout << "Initializing..." << std::endl;

	if (!vv_svd_tc.Initialize(gds, block_size, max_frames_per_svd, patch_padding, island_padding, minimum_normal_similarity))
	{
		std::cout << "PROBLEM CREATING INTERMEDIARY FILE!" << std::endl;
		return;
	}

	std::cout << "Saving final file..." << std::endl;

	if (!vv_svd_tc.SaveFinalFile(intermediary_file, final_file, significant_value_ratio, max_allowed_components))
	{
		std::cout << "PROBLEM CREATING FINAL FILE!" << std::endl;
		return;
	}

	std::cout << "Texturing meshes..." << std::endl;

	if (!vv_svd_tc.AugmentFinalFileWithTextureData(input_folder, mesh_sf, texture_sf, final_file, output_texture_tag, digit_count,
		Eigen::Vector2i(tex_size, tex_size), uv_epsilon, kernel_size, kernel_scale))
	{
		std::cout << "PROBLEM TEXTURING MESHES!" << std::endl;
		return;
	}
	std::cout << "Recreating meshes..." << std::endl;

	if (!vv_svd_tc.ReconstructMeshes(final_file, output_mesh_tag, digit_count))
	{
		std::cout << "PROBLEM RECONSTRUCTING MESHES!" << std::endl;
		return;
	}

#if TSVD_TIME_LOGGING
	vv_svd_tc.CloseTimeLogFile();
#endif
}

void VV_SVD_TemporalSuite::TestFromPreExistingIntermediaryWithoutTexturing()
{
	std::filesystem::create_directories(compressed_sequence_folder);
	std::filesystem::create_directories(reconstructed_sequence_folder);

#if TSVD_TIME_LOGGING
	vv_svd_tc.SetTimeLogFile(time_log_path_final);
#endif

	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	std::cout << "Initializing..." << std::endl;

	if (!vv_svd_tc.Initialize(gds, block_size, max_frames_per_svd, patch_padding, island_padding, minimum_normal_similarity))
	{
		std::cout << "PROBLEM CREATING INTERMEDIARY FILE!" << std::endl;
		return;
	}

	std::cout << "Saving final file..." << std::endl;

	if (!vv_svd_tc.SaveFinalFile(intermediary_file, final_file, significant_value_ratio, max_allowed_components))
	{
		std::cout << "PROBLEM CREATING FINAL FILE!" << std::endl;
		return;
	}

	std::cout << "Recreating meshes..." << std::endl;

	if (!vv_svd_tc.ReconstructMeshes(final_file, output_mesh_tag, digit_count))
	{
		std::cout << "PROBLEM RECONSTRUCTING MESHES!" << std::endl;
		return;
	}

#if TSVD_TIME_LOGGING
	vv_svd_tc.CloseTimeLogFile();
#endif
}

void VV_SVD_TemporalSuite::run(int argc, char** argv)
{
	//TestSingleSequence();

	TestSingleIntermediary();

	//TestFromPreExistingIntermediary();
	
	//TestFromPreExistingIntermediaryWithoutTexturing();
}
