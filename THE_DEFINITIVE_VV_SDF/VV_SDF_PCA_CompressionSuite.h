#pragma once

#include "TestSuite.h"

#include "SDF_PCA_Compressor.h"

class VV_SDF_PCA_CompressionSuite : public TestSuite
{
	const size_t grid_width_voxels = 128;
	const double grid_width_meters = 1.6;
	const Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);

	//const Eigen::Vector4i pca_reach = Eigen::Vector4i(2, 2, 2, 64);
	//const Eigen::Vector4i pca_reach = Eigen::Vector4i(2, 2, 2, 32);
	const Eigen::Vector4i pca_reach = Eigen::Vector4i(1, 1, 1, 512);
	//const Eigen::Vector4i pca_reach = Eigen::Vector4i(4, 4, 4, 64);
	//const Eigen::Vector3i block_size = Eigen::Vector3i(4, 4, 4);
	const Eigen::Vector3i block_size = Eigen::Vector3i(8, 8, 8);
	//const Eigen::Vector3i block_size = Eigen::Vector3i(16, 16, 16);

	const double shell_size = 0.04;
	const double mesh_maximum_artifact_size = 0.05;

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

	GridDataStruct gds;

	SDF_PCA_Compressor compressor;

	std::string test_seq_folder = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch";
	std::string intermediary_file = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/AB_PUNCH_PIECEWISE_SDF/Test_Prl.prl";
	std::string pca_file = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/AB_PUNCH_PCA/Test_Pcf.pcf";
	std::string reconstruction_folder = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ReconstructedSequences/_RECONSTRUCTED_AB-2PUNCH_TSVD";
	std::string reconstruction_tag = "AB-2PUNCH_RECONSTR_TSVD_";

	std::string intermediary_reconstruction_folder = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ReconstructedSequences/_RECONSTRUCTED_PIECEWISE_NO_SVD";
	std::string intermediary_reconstruction_tag = "AB-2PUNCH_RECONSTR_NO_SVD_";

	bool InitializeCompressor();

	void TestCreateIntermediary();

	void TestRecreateMeshesFromIntermediary();

	void TestCreatePCA();

	void TestSingleReconstruction();

	void TestSpecificSegmentBeforeAndAfterRLE(Eigen::Vector4i location, Eigen::Vector4i span);
public:
	void run(int argc, char** argv);
};