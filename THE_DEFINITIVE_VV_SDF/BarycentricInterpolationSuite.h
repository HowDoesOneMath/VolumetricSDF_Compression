#pragma once

#include "TestSuite.h"
#include "BasicGeometry.h"

#include "VV_Mesh.h"
#include "VV_CGAL_Marshaller.h"

#include <string>

#include <CImg.h>

class BarycentricInterpolationSuite : public TestSuite
{
	std::string output_image = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/TestTriangle.png";

	cimg_library::CImg<unsigned char> image;

	Eigen::Vector2d p0 = Eigen::Vector2d(0, 0);
	Eigen::Vector2d p1 = Eigen::Vector2d(1, 0);
	Eigen::Vector2d p2 = Eigen::Vector2d(0, 1);

	VV_Mesh test_mesh;
	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;

	size_t image_size = 64;

	void TestInterpolation();
	
	void TestCGAL();

public:
	void run(int argc, char** argv);
};