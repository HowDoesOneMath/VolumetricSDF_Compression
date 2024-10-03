#pragma once

#include "TestSuite.h"

#include <vector>
#include <set>

#include <Eigen/Geometry>
#include <CImg.h>

#include <iostream>
#include <string>

#include "RectanglePacker.h"
#include "RandomWrapper.h"

class RectanglePackingSuite : public TestSuite
{
	std::vector<std::pair<Eigen::Vector2i, Eigen::Vector2i>> initial_rectangles;

	size_t rectangle_count = 64;

	std::string rect_save_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/PackingRect.png";
	cimg_library::CImg<unsigned char> output_rect;

	Eigen::Vector2i rectangle_random_size_bounds = Eigen::Vector2i(1, 32);
	Eigen::Vector2i rectangle_random_starting_points = Eigen::Vector2i(0, 32);

	Eigen::Vector2i min_bound;
	Eigen::Vector2i max_bound;

	RandomWrapper rw;

	RectanglePacker<int> rp;

	void GenerateRectanglesRandom();
	void GenerateRectanglesTriangle();
	void GenerateRectanglesSquare();

	void PrintRectangleStats();

	void PackRectangles();

	void OutputCImgRect();

public:
	void run(int argc, char** argv);
};