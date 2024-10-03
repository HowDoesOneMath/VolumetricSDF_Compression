#pragma once

#include "TestSuite.h"

#include "VV_TSDF.h"
#include "VV_Mesh.h"
#include "MeshPartition.h"

#include <chrono>

class UnsignedFieldSuite : public TestSuite
{
	VV_TSDF test_TSDF;
	VV_Mesh test_mesh;

	//std::string mesh_to_approximate = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/Monkey.obj";
	//std::string mesh_to_approximate = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/Damaged_Monkey.obj";

	std::string mesh_to_approximate = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	//std::string texture_to_remap = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/tex_AB-2punch_0000001.jpg";

	//std::string output_mesh_without_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/EXPANDED_Monkey";
	//std::string shrunk_output_mesh_without_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/SHRUNK_Monkey";

	std::string output_mesh_without_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/EXPANDED_Puncher";
	std::string shrunk_output_mesh_without_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/SHRUNK_Puncher";

	std::string output_texture = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/SHRUNK_Puncher_TEX";


	std::string single_triangle = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/OneTriangle.obj";

	std::string mesh_file_tag = ".obj";
	//std::string texture_file_tag = ".png";

	MeshPartition mp;

	double center = 0.0;

	Eigen::Vector3d concatenate_offset = Eigen::Vector3d(-1.5, 0, 0);

	const static int density_count = 3;

	size_t grid_densities[density_count] = {
		80,
		100,
		125
	};

	double grid_length = 1.6;

	double buffer_distance = 3;

	double bad_mesh_threshold = 0.04;


	void TestOneMesh();

	void TestOneTriangle();

public:
	void run(int argc, char** argv);
};