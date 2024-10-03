#pragma once

#include "TestSuite.h"

#include "SequenceFinder.h"

#include <vector>
#include <string>

class SequenceFinderSuite : public TestSuite
{
	std::string sf_mesh_key = "Mesh";
	std::string sf_texture_key = "Tex";

	SequenceFinderDetails mesh_sf = SequenceFinderDetails(sf_mesh_key, ".obj");
	SequenceFinderDetails tex_png_sf = SequenceFinderDetails(sf_texture_key, ".png");
	SequenceFinderDetails tex_jpg_sf = SequenceFinderDetails(sf_texture_key, ".jpg");
	SequenceFinder sf;

	std::string input_folder = "D:/_VV_DATASETS/_VOLOGRAMS/RAFA/Rafa_Approves_hd_4k";

	void FindTestSequence();

public:
	void run(int argc, char** argv);
};