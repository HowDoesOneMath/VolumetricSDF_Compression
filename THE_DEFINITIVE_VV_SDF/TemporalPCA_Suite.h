#pragma once

#include "TestSuite.h"

#include "PCA_Encoder.h"
#include "SequenceFinder.h"
#include "MeshPartition.h"
#include "LZ_Encoder.h"

#include "VV_Mesh.h"
#include "VV_TSDF.h"
#include "VV_SaveFileBuffer.h"

#include "AdditionalUtilities.h"

#include <string>
#include <iostream>

#include <Eigen/Dense>

class TemporalPCA_Suite : public TestSuite
{
	std::string sequence_folder = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch";

	std::string pca_comparison_mesh = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/PCA_CopyPuncher_1.obj";

	std::string grouping_output_folder = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ReconstructedSequences/_RECONSTRUCTED_PCA_TEST_GROUPING_V1";
	std::string output_mesh_tag = "MESH_GROUP_";
	std::string output_mesh_ext = ".obj";

	Eigen::Vector3i block_size = Eigen::Vector3i(8,8,8);
	Eigen::Vector3i span_size = Eigen::Vector3i(8,8,8);

	size_t grid_width_voxels = 128;
	double grid_width_meters = 1.6;
	Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinder sf;
	LZ_Encoder lze;

	GridDataStruct gds;
	VV_TSDF sdf;
	VV_Mesh input_mesh;
	double sdf_buffer_distance = 0.04;

	MeshPartition mp;
	double artifact_size = 0.1;

	PCA_Encoder pca_enc;
	//size_t group_size = 1;
	size_t group_size = 8;
	double significance_epsilon = 0.0001;
	double significance_thresh = 1.01; //No energy limit
	//double significance_thresh = 0.99;
	//double significance_thresh = 0.95;
	//double significance_thresh = 0.90;
	//double significance_thresh = 0.50;
	//double significance_thresh = 0.10; //Don't ever use this, this is more or less an error check

	//size_t significance_value_limit = SIZE_MAX; //No value limit
	//size_t significance_value_limit = 1;
	//size_t significance_value_limit = 2;
	//size_t significance_value_limit = 5;
	size_t significance_value_limit = 10;

	//size_t max_frames_to_process = SIZE_MAX;
	size_t max_frames_to_process = 1;
	//size_t max_frames_to_process = 2;
	//size_t max_frames_to_process = 8;


	VV_SaveFileBuffer sfb;

	void LoadSequences();
	void InitializeSDF();
	void InitializePCA_Encoder();
	void ReadAndCleanMesh(std::string to_read, VV_Mesh* mesh, double artifact_size);

	void LoadPCA_Matrix(PCA_Encoder* enc, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size);
	void LoadPCA_MatrixInBlocks(PCA_Encoder* enc, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size);
	void LoadMatrix(Eigen::MatrixXd &mat, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size);
	void EmplaceMatrix(Eigen::MatrixXd &mat, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size);
	void EmplacePCA_MatrixInBlocks(PCA_Encoder* enc, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size);

	void CheckAdjacentFrames();

	void SaveAdjacentFramesWithPCA_Reuse(size_t frame_cap);

	size_t FindSignificantValueCount(Eigen::VectorXd &values, double significance_ratio, double epsilon);

public:
	void run(int argc, char** argv);
};