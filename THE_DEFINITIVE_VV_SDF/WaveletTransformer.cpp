#include "WaveletTransformer.h"

std::shared_ptr<std::vector<double>> WaveletTransformer::HaarInternal(std::vector<double>& input_signal, size_t elem_count)
{
    auto to_return = std::make_shared<std::vector<double>>();

    size_t signal_size_half = elem_count / 2;
    size_t signal_size_half_greater = elem_count - signal_size_half;

    to_return->resize(elem_count);

    for (size_t i = 0; i < signal_size_half; ++i)
    {
        (*to_return)[i] = 0.5 * (input_signal[2 * i] + input_signal[2 * i + 1]);
        (*to_return)[i + signal_size_half_greater] = input_signal[2 * i] - (*to_return)[i];
    }

    if (signal_size_half_greater > signal_size_half)
    {
        (*to_return)[signal_size_half] = input_signal[elem_count - 1];
    }

    if (signal_size_half_greater > 1)
    {
        auto remaining_signal = HaarInternal(*to_return, signal_size_half_greater);

        for (size_t i = 0; i < signal_size_half_greater; ++i)
        {
            (*to_return)[i] = (*remaining_signal)[i];
        }
    }

    return to_return;
}

std::shared_ptr<std::vector<double>> WaveletTransformer::HaarInverseInternal(std::vector<double>& input_signal, size_t elem_count)
{
    auto to_return = std::make_shared<std::vector<double>>();

    size_t signal_size_half = elem_count / 2;
    size_t signal_size_half_greater = elem_count - signal_size_half;

    to_return->resize(elem_count);

    if (elem_count > 1)
    {
        auto remaining_signal = HaarInverseInternal(input_signal, signal_size_half_greater);

        for (size_t i = 0; i < signal_size_half; ++i)
        {
            (*to_return)[2 * i] = (*remaining_signal)[i] + input_signal[i + signal_size_half_greater];
            (*to_return)[2 * i + 1] = (*remaining_signal)[i] - input_signal[i + signal_size_half_greater];
        }

        if (signal_size_half_greater > signal_size_half)
        {
            (*to_return)[elem_count - 1] = (*remaining_signal)[signal_size_half];
        }
    }
    else
    {
        (*to_return)[signal_size_half] = input_signal[0];
    }

    return to_return;
}

std::shared_ptr<std::vector<unsigned char>> WaveletTransformer::LaplacianInternal(std::vector<unsigned char>& input_signal, size_t elem_count)
{
    auto to_return = std::make_shared<std::vector<unsigned char>>();

    size_t signal_size_half = elem_count / 2;
    size_t signal_size_half_greater = elem_count - signal_size_half;

    to_return->resize(elem_count);

    for (size_t i = 0; i < signal_size_half; ++i)
    {
        (*to_return)[i] = input_signal[2 * i];
        (*to_return)[i + signal_size_half_greater] = input_signal[2 * i + 1] - (*to_return)[i];
    }

    if (signal_size_half_greater > signal_size_half)
    {
        (*to_return)[signal_size_half] = input_signal[elem_count - 1];
    }

    if (signal_size_half_greater > 1)
    {
        auto remaining_signal = LaplacianInternal(*to_return, signal_size_half_greater);

        for (size_t i = 0; i < signal_size_half_greater; ++i)
        {
            (*to_return)[i] = (*remaining_signal)[i];
        }
    }

    return to_return;
}

std::shared_ptr<std::vector<unsigned char>> WaveletTransformer::LaplacianInverseInternal(std::vector<unsigned char>& input_signal, size_t elem_count)
{
    auto to_return = std::make_shared<std::vector<unsigned char>>();

    size_t signal_size_half = elem_count / 2;
    size_t signal_size_half_greater = elem_count - signal_size_half;

    to_return->resize(elem_count);

    if (elem_count > 1)
    {
        auto remaining_signal = LaplacianInverseInternal(input_signal, signal_size_half_greater);

        for (size_t i = 0; i < signal_size_half; ++i)
        {
            (*to_return)[2 * i] = (*remaining_signal)[i];
            (*to_return)[2 * i + 1] = (*remaining_signal)[i] + input_signal[i + signal_size_half_greater];
        }

        if (signal_size_half_greater > signal_size_half)
        {
            (*to_return)[elem_count - 1] = (*remaining_signal)[signal_size_half];
        }
    }
    else
    {
        (*to_return)[signal_size_half] = input_signal[0];
    }

    return to_return;
}

std::shared_ptr<std::vector<double>> WaveletTransformer::HaarDecomposition(std::vector<double>& input_signal)
{
    auto to_return = HaarInternal(input_signal, input_signal.size());

    return to_return;
}

std::shared_ptr<std::vector<double>> WaveletTransformer::HaarRecomposition(std::vector<double>& input_signal)
{
    auto to_return = HaarInverseInternal(input_signal, input_signal.size());

    return to_return;
}

std::shared_ptr<std::vector<unsigned char>> WaveletTransformer::LaplacianDecomposition(std::vector<unsigned char>& input_signal)
{
    auto to_return = LaplacianInternal(input_signal, input_signal.size());

    return to_return;
}

std::shared_ptr<std::vector<unsigned char>> WaveletTransformer::LaplacianRecomposition(std::vector<unsigned char>& input_signal)
{
    auto to_return = LaplacianInverseInternal(input_signal, input_signal.size());

    return to_return;
}
