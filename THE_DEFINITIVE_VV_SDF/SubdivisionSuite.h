#pragma once

#include "TestSuite.h"

#include "VV_Mesh.h"

#include <string>

class SubdivisionSuite : public TestSuite
{
	VV_Mesh to_subdivide;
	int iterations = 1;

	std::string mesh_input = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingCube.obj";
	std::string mesh_output = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingCubeSUBDIV.obj";

	void SubdivideTestMesh();
public:
	void run(int argc, char** argv);
};