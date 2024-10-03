#pragma once

#include "TestSuite.h"

#include <string>

#include <UVAtlas.h>
#include "VV_Mesh.h"

class UVAtlasSuite : public TestSuite
{
	std::string input_mesh = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/Monkey.obj";
	std::string output_mesh = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/MonkeyNewUVs.obj";

	VV_Mesh vv_mesh;

	void RecreateUVs();

public:
	void run(int argc, char** argv);
};