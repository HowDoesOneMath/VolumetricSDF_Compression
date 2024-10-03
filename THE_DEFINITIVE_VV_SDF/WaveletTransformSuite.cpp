#include "WaveletTransformSuite.h"



std::shared_ptr<std::vector<double>> WaveletTransformSuite::CreateTestWavelet()
{
	auto to_return = std::make_shared<std::vector<double>>();

	for (size_t i = 0; i < test_wavelet_size; ++i)
	{
		to_return->push_back(i % 5);
	}

	return to_return;
}

std::shared_ptr<std::vector<unsigned char>> WaveletTransformSuite::CreateTestWaveletQuantized()
{
	auto to_return = std::make_shared<std::vector<unsigned char>>();

	for (size_t i = 0; i < test_wavelet_size; ++i)
	{
		to_return->push_back(i % 5);
	}

	return to_return;
}

void WaveletTransformSuite::TestEncodeDecode()
{
	auto signal = CreateTestWavelet();

	auto encoded_signal = wave_trans.HaarDecomposition(*signal);

	auto decoded_signal = wave_trans.HaarRecomposition(*encoded_signal);

	std::cout << "INPUT SIGNAL: " << std::endl;

	for (size_t i = 0; i < signal->size(); ++i)
	{
		std::cout << "\t" << i << ": " << (*signal)[i] << std::endl;
	}

	std::cout << "ENCODED SIGNAL: " << std::endl;

	for (size_t i = 0; i < signal->size(); ++i)
	{
		std::cout << "\t" << i << ": " << (*encoded_signal)[i] << std::endl;
	}

	std::cout << "DECODED SIGNAL: " << std::endl;

	for (size_t i = 0; i < signal->size(); ++i)
	{
		std::cout << "\t" << i << ": " << (*decoded_signal)[i] << std::endl;
	}
}

void WaveletTransformSuite::TestEncodeDecodeQuantized()
{
	auto signal = CreateTestWaveletQuantized();

	auto encoded_signal = wave_trans.LaplacianDecomposition(*signal);

	auto decoded_signal = wave_trans.LaplacianRecomposition(*encoded_signal);

	std::cout << "INPUT SIGNAL: " << std::endl;

	for (size_t i = 0; i < signal->size(); ++i)
	{
		std::cout << "\t" << i << ": " << (int)(*signal)[i] << std::endl;
	}

	std::cout << "ENCODED SIGNAL: " << std::endl;

	for (size_t i = 0; i < signal->size(); ++i)
	{
		std::cout << "\t" << i << ": " << (int)(*encoded_signal)[i] << std::endl;
	}

	std::cout << "DECODED SIGNAL: " << std::endl;

	for (size_t i = 0; i < signal->size(); ++i)
	{
		std::cout << "\t" << i << ": " << (int)(*decoded_signal)[i] << std::endl;
	}
}

void WaveletTransformSuite::run(int argc, char** argv)
{
	//TestEncodeDecode();
	TestEncodeDecodeQuantized();
}
