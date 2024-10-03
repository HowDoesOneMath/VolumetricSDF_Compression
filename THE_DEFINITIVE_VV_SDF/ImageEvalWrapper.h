#pragma once

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

class ImageEvalWrapper
{
	AVFrame* frame1 = nullptr;
	AVFrame* frame2 = nullptr;

	void CImgToFFMPEG(cimg_library::CImg<unsigned char>& src, AVFrame* dst);

	bool InitializeFrame(AVFrame** to_init, int width, int height);
public:
	bool Initialize(int width, int height);
	double GetSSIM(cimg_library::CImg<unsigned char>& img1, cimg_library::CImg<unsigned char>& img2);
	double GetVAMF(cimg_library::CImg<unsigned char>& img);
	void CleanUp();
};