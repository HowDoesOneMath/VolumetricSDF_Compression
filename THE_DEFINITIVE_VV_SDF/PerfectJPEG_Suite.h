#pragma once

#include "TestSuite.h"

#include <CImg.h>
#include <string>

class PerfectJPEG_Suite : public TestSuite
{
	std::string input_png = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_Separated_8x8x8.png";
	std::string output_jpeg = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_Separated_8x8x8.jpg";

	void TurnPNG_ToPerfectJPEG(std::string inpng, std::string outjpg);

public:
	void run(int argc, char** argv);
};