#pragma once

#include "TestSuite.h"

#include <string>
#include "VV_Mesh.h"
#include "MeshPartition.h"

class VV_Mesh_Suite : public TestSuite
{
	std::string input_file = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/Monkey.obj";
	std::string output_file = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/SameMonkey.obj";

	//std::string partition_test = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/Monkey.obj";
	//std::string partition_test = "D:/_VV_DATASETS/RAFA/Rafa_Approves_hd_4k/OBJ/Frame_00001_textured_hd_t_s_c.obj";
	std::string partition_test = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";

	std::string partition_output = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/ArtifactRemovedPUNCH.obj";

	double bad_mesh_threshold = 0.01;

	VV_Mesh test_mesh;

	MeshPartition partition;

	void PartitionMesh();

	void ReadWrite();

public:
	void run(int argc, char** argv);
};