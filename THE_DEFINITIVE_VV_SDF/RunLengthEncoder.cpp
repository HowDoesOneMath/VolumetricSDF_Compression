#include "RunLengthEncoder.h"

void RunLengthEncoder::SaveToBuffer(VV_SaveFileBuffer& buffer)
{
	buffer.WriteArrayToBuffer(lengths.data(), lengths.size());
	buffer.WriteArrayToBuffer(runs.data(), runs.size());
}

void RunLengthEncoder::LoadFromBuffer(VV_SaveFileBuffer& buffer, size_t target_lengths_sum)
{
	runs.clear();
	lengths.clear();

	size_t total_lengths = 0;
	size_t runs_to_read = 0;

	size_t is_unique;
	size_t actual_length;

	while (total_lengths < target_lengths_sum)
	{
		lengths.push_back(0);
		buffer.ReadObjectFromBuffer(lengths.back());
		actual_length = lengths.back() & remove_unique_and_negative;

		is_unique = lengths.back() & signal_unique;
		runs_to_read += (is_unique > 0) * actual_length + (is_unique == 0);

		total_lengths += actual_length;
	}

	runs.resize(runs_to_read);
	buffer.ReadArrayFromBuffer(runs.data(), runs.size());
}

size_t RunLengthEncoder::GetTotalLength()
{
	size_t total_length = 0;

	for (size_t i = 0; i < lengths.size(); ++i)
	{
		total_length += lengths[i] & remove_unique_and_negative;
	}

	return total_length;
}

void RunLengthEncoder::ResizeRunsAccordingToLengths()
{
	size_t runs_size = 0;

	for (size_t i = 0; i < lengths.size(); ++i)
	{
		if (lengths[i] & signal_unique)
		{
			runs_size += (lengths[i] & remove_unique_and_negative);
		}
		else
		{
			runs_size += 1;
		}
	}

	runs.resize(runs_size);
}

void RunLengthEncoder::DebugRunLengths()
{
	size_t length;
	size_t mask = ~(signal_negative | signal_unique);

	size_t parsed = 0;
	size_t run_location = 0;

	unsigned char run_value;

	std::cout << "Summary:\n\t";

	for (size_t i = 0; i < lengths.size(); ++i)
	{
		length = lengths[i] & remove_unique_and_negative;
		 std::cout << length;

		if (lengths[i] & signal_unique)
		{
			std::cout << " (UNIQUE) :";

			for (size_t j = 0; j < length; ++j)
			{
				std::cout << " " << (int)runs[run_location + j];
			}
			
			run_location += length;
		}
		else
		{
			std::cout << " (SAME) :";

			run_value = runs[run_location] & value_mask;

			if (((lengths[i] & signal_negative) > 0) != (run_value <= negative_check))
			{
				std::cout << " " << (int)((runs[run_location] & ~value_mask) | (value_mask - (runs[run_location] & value_mask)));
			}
			else
			{
				std::cout << " " << (int)runs[run_location];
			}

			++run_location;
		}

		std::cout << " \t";

		parsed += length;
	}

	std::cout << std::endl;
}

void RunLengthEncoder::GetRunLength(unsigned char* data, size_t data_amount)
{
	runs.clear();
	lengths.clear();

	size_t starting_point = 0;
	size_t read_values = 0;
	size_t last_unique_byte = 0;
	size_t last_unique_byte_minus_starting_point = 0;

	size_t sizeof_length = sizeof(size_t);

	//bool is_unique = false;

	for (size_t i = 0; i < data_amount; ++i)
	{
		if (data[i] != data[last_unique_byte])
		{
			last_unique_byte = i;
			last_unique_byte_minus_starting_point = last_unique_byte - starting_point;
		}

		++read_values;

		if (read_values - last_unique_byte_minus_starting_point > sizeof_length)
		{
			if (last_unique_byte_minus_starting_point > 0)
			{
				runs.insert(runs.end(), &data[starting_point], &data[last_unique_byte]);
				lengths.push_back(last_unique_byte_minus_starting_point | signal_unique);
			}

			while (data[i] == data[last_unique_byte])
			{
				++i;

				if (i >= data_amount)
				{
					break;
				}
			}

			runs.push_back(data[last_unique_byte]);
			lengths.push_back((i - last_unique_byte) | ((data[last_unique_byte] & value_mask) <= negative_check ? signal_negative : 0));

			starting_point = i;
			last_unique_byte = i;
			read_values = 0;
			last_unique_byte_minus_starting_point = 0;

			--i;
		}
	}

	if (starting_point < last_unique_byte)
	{
		runs.insert(runs.end(), &data[starting_point], &data[data_amount]);
		lengths.push_back((data_amount - starting_point) | signal_unique);
	}
	else if (last_unique_byte < data_amount)
	{
		runs.insert(runs.end(), &data[last_unique_byte], &data[data_amount]);
		lengths.push_back((data_amount - last_unique_byte) | ((data[last_unique_byte] & value_mask) <= negative_check ? signal_negative : 0));
	}
}

void RunLengthEncoder::ExtractOriginal(unsigned char* data)
{
	size_t length;

	size_t parsed = 0;
	size_t run_location = 0;

	unsigned char run_value;

	for (size_t i = 0; i < lengths.size(); ++i)
	{
		length = lengths[i] & remove_unique_and_negative;

		if (lengths[i] & signal_unique)
		{
			memcpy(&(data[parsed]), &(runs[run_location]), length);
			run_location += length;
		}
		else
		{
			run_value = runs[run_location] & value_mask;

			if (((lengths[i] & signal_negative) > 0) != (run_value <= negative_check))
			{
				memset(&(data[parsed]), (runs[run_location] & ~value_mask) | (value_mask - (runs[run_location] & value_mask)), length);
			}
			else
			{
				memset(&(data[parsed]), runs[run_location], length);
			}

			++run_location;
		}

		parsed += length;
	}
}
