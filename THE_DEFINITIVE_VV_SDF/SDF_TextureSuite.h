#pragma once

#include "TestSuite.h"

#include "VV_TSDF.h"
#include "VV_Mesh.h"

#include "TextureRemapper.h"

class SDF_TextureSuite : public TestSuite
{
	VV_TSDF sdf;

	TextureRemapper tr;

	std::string mesh_to_approximate = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	std::string texture_to_approximate = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";

public:
	void run(int argc, char** argv);
};