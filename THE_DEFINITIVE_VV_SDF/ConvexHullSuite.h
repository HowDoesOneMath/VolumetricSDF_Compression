#pragma once

#include "TestSuite.h"

#include "BasicGeometry.h"

#include "ConvexHullCreator.h"
#include "SDF_RotationCaliper.h"

#include "VV_Mesh.h"

#include <CImg.h>
#include <math.h>

#include "CImgAdditionalDrawingFunctionality.h"

class ConvexHullSuite : public TestSuite
{
	const double J_PI = 3.14159265358979323846;

	//This is not a real mesh, only existing to load uv coordinates into the hull algorithm.
	VV_Mesh test_mesh;

	ConvexHullCreator chc;
	size_t test_density = 55;
	size_t img_size = 512;

	double line_thickness = 3.0;

	SDF_RotationCaliper rc;

	std::string test_output_image = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/ConvexHull.png";

	std::shared_ptr<std::vector<Eigen::Vector2d>> ConstructRoughBoundingBox(VV_Mesh& input_mesh, std::vector<size_t> &hull_points, size_t target_edge);

	void QuickGenerateIndicesUVs(VV_Mesh &mesh);

	void CreateTestPoints_TwoSinWaves(size_t density);

	void CreateTestPoints_SinFlower(size_t density);

	void TestConvexHull();

	void TestAutomaticOrientation();

	void TestFindPerpendiculars();

public:
	void run(int argc, char** argv);
};