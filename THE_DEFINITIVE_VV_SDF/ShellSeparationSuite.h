#pragma once

#include "TestSuite.h"

#include "VV_TSDF.h"
#include "VV_Mesh.h"
#include "MeshPartition.h"
#include "SDF_RotationCaliper.h"

class ShellSeparationSuite : public TestSuite
{
	std::string input_mesh_name = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	std::string output_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-2punch_PATCH_DEMO.obj";

	VV_TSDF sdf;
	VV_Mesh input_mesh;
	MeshPartition mp;
	SDF_RotationCaliper rc;

	const size_t grid_width_voxels = 128;
	const double grid_width_meters = 1.6;
	const Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);

	const Eigen::Vector3i block_size = Eigen::Vector3i(8, 8, 8);

	const double shell_size = 0.04;
	const double mesh_maximum_artifact_size = 0.05;

	double minimum_normal_similarity = 0.5;

	Eigen::Vector3d concatenation_offset = Eigen::Vector3d(1.5, 0.0, 0.0);
	double block_spacing_diff = 0.03;
	double normal_spacing_diff = 0.015;


	void SeparateShell();
public:
	void run(int argc, char** argv);
};