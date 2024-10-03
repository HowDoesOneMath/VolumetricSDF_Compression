#pragma once

#include "TestSuite.h"

#include <string>

#include "VV_SaveFileBuffer.h"
#include "PCA_Encoder.h"

class EigenFileSavingSuite : public TestSuite
{
	std::string test_file_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_TempFilesToDebugFeatures/MatrixBinaryTestFile.txt";
	VV_SaveFileBuffer sfb;

	Eigen::MatrixXd test_mat;
	size_t rows = 4;
	size_t cols = 4;

	void LoadTestMatrix(std::string filename);
	void SaveTestMatrix(std::string filename);
public:
	void run(int argc, char** argv);
};