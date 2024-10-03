#pragma once

#include "TestSuite.h"

#include "BasicGeometry.h"

#include <CImg.h>
#include <string>
#include <iostream>

#include <Eigen/Geometry>

#include "VV_TSDF.h"
#include "VV_Mesh.h"

//Yes, this is for linecasts - but still.
class RaycastSuite : public TestSuite
{
	const double pi = 3.14159265358979323846;
	const double to_radians = pi / 180.0;

	std::string input_mesh_name = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	std::string output_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/Ab-2punch_RAYCAST.obj";

	std::string output_texture_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/RaycastTesting.png";
	cimg_library::CImg<unsigned char> raycast_canvas;
	size_t pixels_w = 1024;
	size_t pixels_h = 1024;
	double space_width = 2.0;
	double space_height = (space_width * pixels_h) / pixels_w;

	double colour_shift = 0.5;

	Eigen::Vector3d p0 = Eigen::Vector3d(1, 1, 2);
	Eigen::Vector3d p1 = Eigen::Vector3d(2, 1, 1);
	Eigen::Vector3d p2 = Eigen::Vector3d(1, 2, 1);

	Eigen::Vector3d ray_origin = Eigen::Vector3d(0, 0, 0);

	Eigen::Affine3d transform = Eigen::Affine3d::Identity();
	Eigen::Vector3d translation = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d euler_rotation = Eigen::Vector3d(-45, 60, 0);

	const size_t grid_width_voxels = 128;
	const double grid_width_meters = 1.2;
	const Eigen::Vector3d center = Eigen::Vector3d(0, -0.5, 0);

	VV_TSDF sdf;
	VV_Mesh mesh;

	void InitializeTransform();

	void TestMultiCast();

	void TestSingleCast();

	bool InitializeSDF();

	void TestMesh();

public:
	void run(int argc, char** argv);
};