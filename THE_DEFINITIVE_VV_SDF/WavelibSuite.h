#pragma once

#include "TestSuite.h"

#include "WaveletEncoder.h"

#include "RandomWrapper.h"

#include <vector>

class WavelibSuite : public TestSuite
{
	WaveletEncoder we;

	RandomWrapper rw;

	const int signal_size = 32;

	const int decomposition_count = 2;

	std::vector<double> test_signal;
	std::vector<double> output_signal;

	void GetSimpleWavelet();
public:
	void run(int argc, char** argv);
};