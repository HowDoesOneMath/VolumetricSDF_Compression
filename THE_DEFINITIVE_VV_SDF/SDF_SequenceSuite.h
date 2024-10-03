#pragma once

#include <string>

#include "VV_SaveFileBuffer.h"
#include "RunLengthEncoder.h"

#include "MeshPartition.h"

#include "TextureRemapper.h"

#include "VV_TSDF.h"
#include "VV_Mesh.h"
#include "SequenceFinder.h"
#include "TestSuite.h"

class SDF_SequenceSuite : public TestSuite
{
	std::string sequence_folder = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch";
	std::string save_file = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/AB_PUNCH_SDF/AB_PUNCH_SDF.rlf";

	std::string mesh_file_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_UncompressedSequences/AB-2punch_SDF_MESH_";
	std::string texture_file_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_UncompressedSequences/AB-2punch_SDF_TEX_";
	std::string mesh_file_ext = ".obj";
	std::string texture_file_ext = ".png";

	int number_fixed_length = 6;

	GridDataStruct gds;
	size_t width_voxels = 128;
	double width_meters = 1.6;
	double center_meters = 0.0;

	double unsigned_buffer_distance = 0.04;

	Eigen::Vector2i partition_estimate = Eigen::Vector2i(256, 256);
	double partition_buffer = 0.15;

	VV_SaveFileBuffer buffer;
	RunLengthEncoder rle;
	TextureRemapper tr;
	MeshPartition mp;
	double artifact_threshold = 0.1;

	VV_Mesh input_mesh;
	VV_TSDF sdf;

	cimg_library::CImg<unsigned char> original_tex;
	cimg_library::CImg<unsigned char> remapped_tex;
	Eigen::Vector2i remapped_texture_dims = Eigen::Vector2i(2048, 2048);

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinderDetails texture_sf = SequenceFinderDetails("Texture", ".jpg");
	SequenceFinder sf;

	void InitializeGrid();

	void LoadSequenceNames();
	void SaveWholeSequence();

	void SaveSequenceAsPureMeshes();

public:
	void run(int argc, char** argv);
};