#include "TexturePaddingSuite.h"

void TexturePaddingSuite::LazyAssignOccupancy(cimg_library::CImg<unsigned char>& target_texture, std::vector<bool>& occupancy)
{
	occupancy.clear();
	occupancy.resize(target_texture.width() * target_texture.height(), false);

	for (size_t i = 0; i < target_texture.width(); ++i)
	{
		for (size_t j = 0; j < target_texture.height(); ++j)
		{
			if ((target_texture(i, j, 0, 0) > 0) ||
				(target_texture(i, j, 0, 1) > 0) ||
				(target_texture(i, j, 0, 2) > 0))
			{
				occupancy[i * target_texture.height() + j] = true;
			}
		}
	}
}

void TexturePaddingSuite::ConstructTestMap(cimg_library::CImg<unsigned char>& target_texture, std::vector<bool>& occupancy, 
	size_t att_size, size_t att_edge, size_t att_checkerboard_size)
{
	target_texture.assign(att_size, att_size, 1, 3);
	target_texture.fill(0);

	occupancy.clear();
	occupancy.resize(att_size * att_size, false);

	size_t end_point = att_size - att_edge;
	double div_ratio = 255.0 / (double)(att_size - 2 * att_edge);

	for (size_t i = att_edge; i < end_point; ++i)
	{
		for (size_t j = att_edge; j < end_point; ++j)
		{
			if (((i / att_checkerboard_size) % 2) == ((j / att_checkerboard_size) % 2))
			{
				target_texture(i, j, 0, 0) = (unsigned char)((i - att_edge) * div_ratio);
				target_texture(i, j, 0, 1) = 127;
				target_texture(i, j, 0, 2) = (unsigned char)((j - att_edge) * div_ratio);
			}

			occupancy[i * att_size + j] = true;
		}
	}
}

void TexturePaddingSuite::run(int argc, char** argv)
{
	//ConstructTestMap(test_texture, occupancy_map, attribute_map_size, attribute_edge_size, att_checkerboard_size);

	test_texture.assign(test_input_texture_name.c_str());
	LazyAssignOccupancy(test_texture, occupancy_map);

	//test_texture.save_png(test_texture_name.c_str());
	padded_texture.assign(test_texture);

	tr.PadTexture(padded_texture, occupancy_map, pad_count);

	padded_texture.append(test_texture, 'x');
	padded_texture.save_png(test_texture_padded_name.c_str());
}

