#include "LZ_Compression_Suite.h"

void LZ_Compression_Suite::TestWithPresetString()
{
	std::vector<unsigned char> buffer;
	std::vector<unsigned char> output_buffer;
	std::string to_output = "if you see this, a problem happened";

	if (!encoder.CompressData((unsigned char*)preset_string.c_str(), preset_string.size(), buffer))
	{
		return;
	}

	std::cout << "Uncompressed Size: " << preset_string.size();
	std::cout << "Compressed Size: " << buffer.size() << std::endl;

	output_buffer.resize(preset_string.size());

	if (!encoder.DecompressData(buffer.data(), buffer.size(), output_buffer))
	{
		return;
	}

	to_output.resize(preset_string.size());
	memcpy(to_output.data(), output_buffer.data(), to_output.size());

	std::cout << "SUCCESS, here's the string:\n" << to_output << std::endl;
}

void LZ_Compression_Suite::run(int argc, char** argv)
{
	TestWithPresetString();
}
