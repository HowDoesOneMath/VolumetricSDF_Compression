#pragma once

#include "TestSuite.h"

#include "RunLengthEncoder.h"
#include "SequenceFinder.h"

#include "PCA_Encoder.h"
#include "VV_Mesh.h"
#include "VV_TSDF.h"

#include "VV_SaveFileBuffer.h"

class PCA_Suite : public TestSuite
{
	std::string test_singular_mesh = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";

	std::string test_sequence = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch";

	std::string test_RLE_output = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/AB_PUNCH_PIECEWISE_SDF/Test_Prl.prl";

	std::string test_PCA_output_mesh = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-2punch_PCA.obj";
	std::string test_before_PCA_output_mesh = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-2punch_SDF.obj";

	GridDataStruct gds;
	size_t square_grid_size = 128;
	size_t grid_block_length = 8;
	double grid_size = 1.6;
	double buffer_size = 0.04;

	Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);

	Eigen::Vector3d mesh_offset = Eigen::Vector3d(-1.5, 0, 0);
	VV_Mesh mesh;
	VV_TSDF sdf;

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinder sf;
	RunLengthEncoder rle;
	VV_SaveFileBuffer piecewise_rle;

	PCA_Encoder pca_enc;

	//double PCA_cull_threshold = 0.01;
	double PCA_cull_threshold = 10.0;

	bool InitializeSDF();
	bool LoadSDF(std::string file_name);
	void SampleGridWhole();

	void AddChunksToEncoder(VV_TSDF& to_encode, size_t block_length_x, size_t block_length_y, size_t block_length_z);
	void ExtractChunksFromEncoder(VV_TSDF& to_decode, size_t block_length_x, size_t block_length_y, size_t block_length_z);

	void SaveGridDividedRLE();
	//TODO: THIS!
	void LoadGridDividedRLE();

	void RedundantSVD();

	void MiniTestSVD();

	void DummyFillGrid();
public:
	void run(int argc, char** argv);
};