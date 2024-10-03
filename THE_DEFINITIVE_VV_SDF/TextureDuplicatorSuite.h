#pragma once

#include "TestSuite.h"

#include <string>

#include <CImg.h>

#include "AdditionalUtilities.h"

class TextureDuplicatorSuite : public TestSuite
{
	std::string input_file_name = "D:/_VV_DATASETS_TRIMMED/_NON_VV_DATASET/Boss_diffuse.png";
	std::string output_file_tag = "D:/_VV_DATASETS_TRIMMED/_NON_VV_DATASET/Boss_diffuse";
	int output_amount = 100;
	int output_digit_count = 4;

	cimg_library::CImg<unsigned char> img;

public:
	void run(int argc, char** argv);
};