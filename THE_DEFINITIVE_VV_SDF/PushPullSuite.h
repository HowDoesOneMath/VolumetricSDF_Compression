#pragma once

#include "TestSuite.h"

#include <CImg.h>
#include <Eigen/Core>

#include <string>
#include <iostream>

#include "TextureRemapper.h"

class PushPullSuite : public TestSuite
{

	std::string input_image_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/TestAtlas.png";
	std::string save_file_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/TestAtlasFilled.png";

	//size_t img_w = 1024;
	//size_t img_h = 1024;
	cimg_library::CImg<unsigned char> test_img;

	size_t kern_w = 3;
	size_t kern_h = 3;
	cimg_library::CImg<double> test_kernel;

	TextureRemapper tr;

	std::shared_ptr<cimg_library::CImg<double>> ConstructTestPattern(int width, int height);

	std::shared_ptr<cimg_library::CImg<double>> RetrieveDoubleImage(cimg_library::CImg<unsigned char>& input_image);
	std::shared_ptr<cimg_library::CImg<unsigned char>> RetrieveByteImage(cimg_library::CImg<double>& input_image);

	void TestPushPull();

	void TestMul();

	void TestConvolveCrop();
public:
	void run(int argc, char** argv);
};