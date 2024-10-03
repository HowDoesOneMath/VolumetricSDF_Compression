#pragma once

#include "TestSuite.h"

#include <string>
#include <vector>
#include <iostream>
#include <CImg.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}


class RawFFMPEG_Suite : public TestSuite
{

public:
	void run(int argc, char** argv);
};