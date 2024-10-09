#pragma once

#include "TestSuite.h"

#include "SequenceFilePathUniversal.h"

#include "BadFileParser.h"

class BadFileParsingSuite : public TestSuite
{
	BadFileParser bfp;

	std::string test_seq = "/SIR_FREDRICK";

	std::string test_folder = GetDatasetsPath() + test_seq;

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");

public:

	void run(int argc, char** argv);
};