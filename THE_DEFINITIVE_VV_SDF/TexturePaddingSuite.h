#pragma once

#include "TestSuite.h"

#include "TextureRemapper.h"

class TexturePaddingSuite : public TestSuite
{
	size_t attribute_map_size = 64;
	size_t attribute_edge_size = 4;

	std::string test_input_texture_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_Separated_8x8x8.png";

	std::string test_texture_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/Unpadded_Texture_64_4.png";
	std::string test_texture_padded_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/Padded_Texture_64_4.png";
	
	cimg_library::CImg<unsigned char> test_texture;
	cimg_library::CImg<unsigned char> padded_texture;
	std::vector<bool> occupancy_map;

	TextureRemapper tr;
	int pad_count = 3;
	size_t att_checkerboard_size = 8;

	void LazyAssignOccupancy(cimg_library::CImg<unsigned char>& target_texture, std::vector<bool>& occupancy);

	void ConstructTestMap(cimg_library::CImg<unsigned char>& target_texture, std::vector<bool>& occupancy,
		size_t att_size, size_t att_edge, size_t att_checkerboard_size);

public:
	void run(int argc, char** argv);
};