#pragma once

#include "TestSuite.h"

#include "SequenceFilePathUniversal.h"

#include "BadFileParser.h"

class BadFileParsingSuite : public TestSuite
{
	BadFileParser bfp;

	//std::string test_seq = "/_NON_VV_DATASET";
	//std::string test_seq = "/AB-2punch";
	//std::string test_seq = "/AB-dodgeLeft";
	//std::string test_seq = "/AB-death";
	//std::string test_seq = "/Basketball";
	//std::string test_seq = "/RAFA";
	//std::string test_seq = "/LEVI";
	std::string test_seq = "/SIR_FREDRICK";

	std::string test_folder = GetDatasetsPath() + test_seq;

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");

public:

	void run(int argc, char** argv);
};