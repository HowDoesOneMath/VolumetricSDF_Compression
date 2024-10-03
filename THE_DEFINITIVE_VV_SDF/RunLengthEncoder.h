#pragma once

#include <vector>
#include <iostream>
#include "VV_SaveFileBuffer.h"

class RunLengthEncoder
{

public:
	std::vector<unsigned char> runs;
	std::vector<size_t> lengths;

	const size_t signal_unique = 0x8000000000000000; //((size_t)1) << 63;
	const size_t signal_negative = 0x4000000000000000; //((size_t)1) << 62;

	const size_t remove_unique_and_negative = 0x3fffffffffffffff;
	const size_t remove_unique = 0x7fffffffffffffff;
	const size_t remove_negative = 0xbfffffffffffffff;

	const unsigned char negative_check = 31;

	const unsigned char value_mask = 63;

	void SaveToBuffer(VV_SaveFileBuffer& buffer);
	void LoadFromBuffer(VV_SaveFileBuffer& buffer, size_t target_lengths_sum);

	size_t GetTotalLength();
	void ResizeRunsAccordingToLengths();

	void DebugRunLengths();

	void GetRunLength(unsigned char* data, size_t data_amount);

	void ExtractOriginal(unsigned char* presized_data);
};