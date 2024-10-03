#pragma once

#include <vector>
#include <iostream>

#include <memory>

class WaveletTransformer
{
	std::shared_ptr<std::vector<double>> HaarInternal(std::vector<double>& input_signal, size_t elem_count);
	std::shared_ptr<std::vector<double>> HaarInverseInternal(std::vector<double>& input_signal, size_t elem_count);

	std::shared_ptr<std::vector<unsigned char>> LaplacianInternal(std::vector<unsigned char>& input_signal, size_t elem_count);
	std::shared_ptr<std::vector<unsigned char>> LaplacianInverseInternal(std::vector<unsigned char>& input_signal, size_t elem_count);

public:
	std::shared_ptr<std::vector<double>> HaarDecomposition(std::vector<double> &input_signal);
	std::shared_ptr<std::vector<double>> HaarRecomposition(std::vector<double>& input_signal);

	std::shared_ptr<std::vector<unsigned char>> LaplacianDecomposition(std::vector<unsigned char>& input_signal);
	std::shared_ptr<std::vector<unsigned char>> LaplacianRecomposition(std::vector<unsigned char>& input_signal);
};