#pragma once

#include "TestSuite.h"
#include "VV_SVD_TemporalCompressor.h"

class VV_SVD_TemporalSuite : public TestSuite
{
	const int digit_count = 6;

	std::string input_folder = "D:/_VV_DATASETS_TRIMMED/AB-2punch";
	//std::string input_folder = "D:/_VV_DATASETS_TRIMMED/AB-2punch";
	//std::string input_folder = "D:/_VV_DATASETS_TRIMMED/AB-2punch";
	std::string sequence_file_identifier = "/AB-2PUNCH";
	//std::string sequence_file_identifier = "/AB-2PUNCH";
	//std::string sequence_file_identifier = "/AB-2PUNCH";

	std::string compressed_sequence_folder = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/_TSVD";
	std::string reconstructed_sequence_folder = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ReconstructedSequences/_TSVD";


	//std::string intermediary_file = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/AB_PUNCH_SVD/INTERMEDIARY_FILE.sif";
	std::string intermediary_file = compressed_sequence_folder + sequence_file_identifier + "/INTERMEDIARY_FILE.sif";
	//std::string final_file = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/AB_PUNCH_SVD/FINAL_FILE.tsvd";
	std::string final_file = compressed_sequence_folder + sequence_file_identifier + "/FINAL_FILE.tsvd";
	//std::string output_mesh_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ReconstructedSequences/_TSVD/AB-2PUNCH/FRAME";
	std::string output_mesh_tag = reconstructed_sequence_folder + sequence_file_identifier + "/FRAME";
	//std::string output_texture_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ReconstructedSequences/_TSVD/AB-2PUNCH/FRAME";
	std::string output_texture_tag = reconstructed_sequence_folder + sequence_file_identifier + "/FRAME";

	const size_t grid_width_voxels = 128;
	const double grid_width_meters = 1.6;
	const Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
	GridDataStruct gds;

	const Eigen::Vector3i block_size = Eigen::Vector3i(8, 8, 8);

	const size_t max_frames_per_svd = 256;

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

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinderDetails texture_sf = SequenceFinderDetails("Texture", ".jpg");

	const int jpg_quality = 100;
	const size_t tex_size = 1024;

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