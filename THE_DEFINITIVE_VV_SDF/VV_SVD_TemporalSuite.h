#pragma once

#include "SequenceFilePathUniversal.h"

#include "TestSuite.h"
#include "VV_SVD_TemporalCompressor.h"

class VV_SVD_TemporalSuite : public TestSuite
{
	const int digit_count = 6;
	
	//const size_t grid_width_voxels = 512;
	//const size_t grid_width_voxels = 256;
	const size_t grid_width_voxels = 128;
	//const size_t grid_width_voxels = 64;
	//const double grid_width_meters = 1.6;
	const double grid_width_meters = 1.1;
	//const Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
	const Eigen::Vector3d center = Eigen::Vector3d(0, 1, 0);
	GridDataStruct gds;

	const Eigen::Vector3i block_size = Eigen::Vector3i(8, 8, 8);

	const size_t max_frames_per_svd = 256;
	//const size_t max_frames_per_svd = 128;
	//const size_t max_frames_per_svd = 64;

	const double shell_size = 0.04;
	const double mesh_maximum_artifact_size = 0.05;

	const double minimum_normal_similarity = 0.5;

	//const double significant_value_ratio = 1.00;
	//const double significant_value_ratio = 0.99;
	//const double significant_value_ratio = 0.95;
	const double significant_value_ratio = 0.90;
	//const double significant_value_ratio = 0.80;
	//const size_t max_allowed_components = 1;
	//const size_t max_allowed_components = 2;
	//const size_t max_allowed_components = 5;
	//const size_t max_allowed_components = 10;
	const size_t max_allowed_components = SIZE_MAX;

	//std::string input_folder = GetDatasetsPath() + "/AB-2punch";
	std::string input_folder = GetDatasetsPath() + "/SIR_FREDRICK";

	//std::string sequence_file_identifier = "/AB-2PUNCH";
	std::string sequence_file_identifier = "/SIR_FREDRICK";

#if ENCODE_SIGN_DATA
	std::string tsvd_code = "/TSVD_PLUS_SIGNS";
#else
	std::string tsvd_code = "/TSVD";
#endif


	std::string significance_identifier = "/SIGNIFICANCE_" + std::to_string((int)(significant_value_ratio * 100 + 0.5));
	std::string grid_voxel_identifier = "/VOXELS_" + std::to_string(grid_width_voxels);
	std::string frames_per_batch_identifier = "/BATCH_" + std::to_string(max_frames_per_svd);

	std::string compressed_sequence_folder = GetCompressionPath() + tsvd_code + grid_voxel_identifier + frames_per_batch_identifier + significance_identifier + sequence_file_identifier;
	std::string reconstructed_sequence_folder = GetReconstructionPath() + tsvd_code + grid_voxel_identifier + frames_per_batch_identifier + significance_identifier + sequence_file_identifier;

	std::string intermediary_file = compressed_sequence_folder + "/INTERMEDIARY_FILE.sif";
	std::string final_file = compressed_sequence_folder + "/FINAL_FILE.tsvd";
	std::string output_mesh_tag = reconstructed_sequence_folder + "/FRAME";

	std::string output_texture_tag = output_mesh_tag;

	std::string time_log_identifier = "/TimeLog.txt";
	std::string time_log_path = compressed_sequence_folder + time_log_identifier;


	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinderDetails texture_sf = SequenceFinderDetails("Texture", ".jpg");

	const int jpg_quality = 100;
	const size_t tex_size = 4096;
	//const size_t tex_size = 2048;
	//const size_t tex_size = 1024;

	const double patch_padding = 0.05;
	const double island_padding = 0.001;

	const double uv_epsilon = 0.0001;

	const int kernel_size = 5;
	const double kernel_scale = 2.0;


	VV_SVD_TemporalCompressor vv_svd_tc;

	void TestSingleSequence();
public:

	void run(int argc, char** argv);
};