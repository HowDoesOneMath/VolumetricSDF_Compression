#include "TextureDuplicatorSuite.h"

void TextureDuplicatorSuite::run(int argc, char** argv)
{
	img.assign(input_file_name.c_str());

	for (int i = 0; i < output_amount; ++i)
	{
		std::string save_name = output_file_tag + GetNumberFixedLength((i + 1), output_digit_count) + ".jpg";
		img.save_jpeg(save_name.c_str(), 100);
	}
}
