#include "WavelibSuite.h"

void WavelibSuite::GetSimpleWavelet()
{
	test_signal.resize(signal_size);

	std::cout << "INPUT SIGNAL: " << std::endl;

	for (int i = 0; i < test_signal.size(); ++i)
	{
		test_signal[i] = (i / 2) % 2;
		std::cout << "\t" << i << "; " << test_signal[i] << std::endl;
	}

	//wt_init

	//we.SetWaveletType("dwt");
	we.Encode(test_signal, output_signal, decomposition_count);

	std::cout << "OUTPUT SIGNAL: " << std::endl;

	for (int i = 0; i < output_signal.size(); ++i)
	{
		std::cout << "\t" << i << "; " << output_signal[i] << std::endl;
	}

	test_signal.clear();
	output_signal.clear();
}

void WavelibSuite::run(int argc, char** argv)
{
	GetSimpleWavelet();
}
