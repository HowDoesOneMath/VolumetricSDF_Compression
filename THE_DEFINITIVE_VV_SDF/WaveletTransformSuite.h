#pragma once

#include "TestSuite.h"
#include "WaveletTransformer.h"

#include <vector>
#include <iostream>

class WaveletTransformSuite : public TestSuite
{
	size_t test_wavelet_size = 17;

	WaveletTransformer wave_trans;

	std::shared_ptr<std::vector<double>> CreateTestWavelet();
	std::shared_ptr<std::vector<unsigned char>> CreateTestWaveletQuantized();

	void TestEncodeDecode();

	void TestEncodeDecodeQuantized();
public:
	void run(int argc, char** argv);
};