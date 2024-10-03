#include "PerfectJPEG_Suite.h"

void PerfectJPEG_Suite::TurnPNG_ToPerfectJPEG(std::string inpng, std::string outjpg)
{
	cimg_library::CImg<unsigned char> test_img;

	test_img.assign(inpng.c_str());

	test_img.save_jpeg(outjpg.c_str(), 100);
}

void PerfectJPEG_Suite::run(int argc, char** argv)
{
	TurnPNG_ToPerfectJPEG(input_png, output_jpeg);
}
