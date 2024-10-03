#pragma once

class MortonOrderer
{
public:
	size_t GetMortonOrder(size_t* inputs, int input_count);

	void GetNormalOrder(size_t* outputs, int output_count, size_t morton_ordered_number);
};