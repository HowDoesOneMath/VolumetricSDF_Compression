#pragma once

#include "TestSuite.h"
#include "VV_Mesh.h"

#include <stdlib.h>
#include <time.h> 

class SemiSplitSuite
{
	std::string input_mesh;

	VV_Mesh mesh;

	void run(int argc, char** argv);
};