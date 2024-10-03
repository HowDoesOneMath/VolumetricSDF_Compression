#pragma once

#include "TestSuite.h"

#include <string>
#include <vector>

#include <CImg.h>
#include "FFMPEG_Wrapper.h"

//#include <ff>

//#include <libavfilter/avfilter.h>

class FFMPEG_Suite : public TestSuite
{
	std::string new_video_file = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_VideoDump/ArbitraryTestVideos/TestOutput.mp4";
	std::string singular_image_tag = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_VideoDump/ArbitraryTestVideos/SingularFrames/test_img_";
	std::string singular_image_ext = ".png";

	std::string video_to_load = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_VideoDump/ArbitraryTestVideos/SingularFrames/command_prompt_test.mp4";

	std::string file_dump_command_line = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_VideoDump/ArbitraryTestVideos/TestOutputFrames/CommandLine/cl_img_";
	std::string file_dump_from_code = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_VideoDump/ArbitraryTestVideos/TestOutputFrames/FromCode/fc_img_";

	cimg_library::CImg<unsigned char> single_frame;
	FFMPEG_Wrapper output_wrapper;
	FFMPEG_Wrapper input_wrapper;

	void FillImageWithTestPattern(int offset, int modulo_value, int r_thresh, int g_thresh, int b_thresh);
	void SaveArbitraryVideo();

	void LoadArbitraryVideo();

public:
	void run(int argc, char** argv);
};