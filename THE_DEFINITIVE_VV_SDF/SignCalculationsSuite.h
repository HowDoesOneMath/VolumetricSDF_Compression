#pragma once

#include "TestSuite.h"

#include "VV_Mesh.h"
#include "VV_TSDF.h"
#include "SequenceFinder.h"
#include "MeshPartition.h"
#include "LZ_Encoder.h"

#include <string>
#include <iostream>

class SignCalculationsSuite : public TestSuite
{
	std::string sequence_folder = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch";

	size_t grid_width_voxels = 128;
	double grid_width_meters = 1.6;
	Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinder sf;

	GridDataStruct gds;
	VV_TSDF sdf;
	VV_Mesh input_mesh;
	double sdf_buffer_distance = 0.04;

	LZ_Encoder lze;

	MeshPartition mp;
	double artifact_size = 0.1;

	void LoadSequences();
	void InitializeSDF();

	void ReadAndCleanMesh(std::string to_read, VV_Mesh* mesh, double artifact_size);
public:
	void run(int argc, char** argv);
};