#pragma once

#include "TestSuite.h"
#include "MortonOrderer.h"

#include <string>
#include <iostream>

#include <CImg.h>

class MortonOrderSuite : public TestSuite
{
	MortonOrderer morder;

	std::string morton_tex_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/MortonTest.png";

	cimg_library::CImg<unsigned char> morton_image;

	void CreateMortonOrderTexture();

public:
	void run(int argc, char** argv);
};