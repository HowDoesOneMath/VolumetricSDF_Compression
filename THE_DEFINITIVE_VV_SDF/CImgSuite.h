#pragma once

#include "TestSuite.h"
#include <CImg.h>

class CImgSuite : public TestSuite
{
	void SampleCImg();
public:
	void run(int argc, char** argv);
};