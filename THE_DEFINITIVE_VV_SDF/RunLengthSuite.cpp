#include "RunLengthSuite.h"

void RunLengthSuite::run(int argc, char** argv)
{

	//std::string test_data;

	std::vector<unsigned char> test_nums = {0, 1, 2, 3, 4};
	std::vector<size_t> test_lengths = {66, 4, 5, 4, 50};

	std::vector<unsigned char> total_data;
	std::vector<unsigned char> reconstructed_data;

	for (int i = 0; i < test_lengths.size(); ++i)
	{
		for (int j = 0; j < test_lengths[i]; ++j)
		{
			total_data.push_back(test_nums[i]);
		}
	}

	reconstructed_data.resize(total_data.size());

	std::cout << "UNCOMPRESSED LENGTH: " << (total_data.size()) << std::endl;

	rle.GetRunLength(total_data.data(), total_data.size());

	rle.DebugRunLengths();

	size_t run;

	size_t current_value = 0;

	for (int i = 0; i < rle.lengths.size(); ++i)
	{
		run = rle.lengths[i] & ~(rle.signal_negative | rle.signal_unique);

		if (rle.lengths[i] & rle.signal_unique)
		{
			std::cout << "UNIQUE " << run << " -> ";

			for (int j = 0; j < run; ++j, ++current_value)
			{
				std::cout << (int)rle.runs[current_value] << "; ";
			}

			std::cout << std::endl;
		}
		else
		{
			if (rle.lengths[i] & rle.signal_negative)
			{
				std::cout << "NEGREP " << run << " -> " << (int)rle.runs[current_value] << std::endl;
			}
			else
			{
				std::cout << "POSREP " << run << " -> " << (int)rle.runs[current_value] << std::endl;
			}

			++current_value;
		}
	}

	std::cout << "COMPRESSED LENGTH: " << (rle.lengths.size() * sizeof(size_t) + rle.runs.size()) << std::endl;

	rle.ExtractOriginal(reconstructed_data.data());

	for (int i = 0; i < total_data.size(); ++i)
	{
		std::cout << (int)total_data[i] << ", ";
	}

	std::cout << std::endl;

	for (int i = 0; i < reconstructed_data.size(); ++i)
	{
		std::cout << (int)reconstructed_data[i] << ", ";
	}

	std::cout << std::endl;
}
