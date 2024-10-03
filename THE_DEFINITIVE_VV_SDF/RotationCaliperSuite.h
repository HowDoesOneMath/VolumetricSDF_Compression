#pragma once

#include "TestSuite.h"

#include <string>
#include <iostream>

#include "VV_TSDF.h"
#include "MeshPartition.h"

#include "SDF_RotationCaliper.h"
#include "TextureRemapper.h"

class RotationCaliperSuite : public TestSuite
{
	std::string test_mesh = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	std::string test_texture = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/tex_AB-2punch_0000001.jpg";

	std::string output_mesh = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_Separated_8x8x8.obj";
	std::string output_texture = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_Separated_8x8x8.png";

	const size_t grid_width_voxels = 128;
	const double grid_width_meters = 1.6;
	const Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);

	const Eigen::Vector3i block_size = Eigen::Vector3i(8, 8, 8);

	const double shell_size = 0.04;
	const double mesh_maximum_artifact_size = 0.05;

	double minimum_normal_similarity = 0.5;

	Eigen::Vector3d offset_amount = Eigen::Vector3d(0.8, 0.0, 0.0);

	size_t attribute_map_size = 1024;
	double approximate_pixel_uv_overlap = 1.0;
	int kernel_size = 5;
	double kernel_scale = 2.0;

	VV_Mesh mesh;
	cimg_library::CImg<unsigned char> texture;
	VV_TSDF sdf;
	MeshPartition mp;
	SDF_RotationCaliper rc;

	TextureRemapper tr;

	Eigen::Vector2d test_point_0 = Eigen::Vector2d(0, 0);
	Eigen::Vector2d test_point_1 = Eigen::Vector2d(1, 1);
	Eigen::Vector2d test_point_2 = Eigen::Vector2d(2, 1);
	Eigen::Vector2d test_point_3 = Eigen::Vector2d(2, -1);

	void TestSingleRotation();

	void TestBlockPartition();

public:
	void run(int argc, char** argv);
};