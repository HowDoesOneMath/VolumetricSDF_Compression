#pragma once

#include <lz4.h>
#include <string>
#include <vector>
#include <iostream>

#include "VV_SaveFileBuffer.h"

class LZ_Encoder
{
public:
	bool CompressData(unsigned char* data, size_t data_size, std::vector<unsigned char> &output_vector);

	bool DecompressData(unsigned char* data, size_t data_size, std::vector<unsigned char>& presized_output_vector);

	size_t SaveToBuffer(VV_SaveFileBuffer& sfb, unsigned char* data, size_t data_size);

	size_t ReadFromBuffer(VV_SaveFileBuffer& sfb, unsigned char* presized_data, size_t data_size);
};