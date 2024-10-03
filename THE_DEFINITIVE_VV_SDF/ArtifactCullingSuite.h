#pragma once

#include "TestSuite.h"

#include "VV_TSDF.h"
#include "VV_Mesh.h"
#include "MeshPartition.h"

class ArtifactCullingSuite : public TestSuite
{
	std::string input_mesh_name = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	std::string output_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-2punch_ArtifactComparison.obj";

	MeshPartition mp;
	VV_TSDF sdf;
	VV_Mesh input_mesh;

	const size_t grid_width_voxels = 128;
	const double grid_width_meters = 1.6;
	const Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);

	const double shell_size = 0.04;
	const double mesh_maximum_artifact_size = 0.05;

	Eigen::Vector3d offset_amount = Eigen::Vector3d(0.8, 0.0, 0.0);

	void CreateComparisonMesh();
public:
	void run(int argc, char** argv);
};