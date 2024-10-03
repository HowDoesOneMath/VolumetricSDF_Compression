#pragma once

#include "TestSuite.h"

#include "VV_TSDF.h"

class MC_TestSuite : public TestSuite
{
	VV_TSDF tsdf;

	std::string mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/A_Sphere.obj";

	size_t grid_len = 51;
	double grid_span = 2.0;
public:
	void run(int argc, char** argv);
};