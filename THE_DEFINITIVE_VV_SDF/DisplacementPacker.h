#pragma once

#include "VV_SaveFileBuffer.h"
#include "MortonOrderer.h"

#include "VV_Mesh.h"

#include <CImg.h>

#include <string>
#include <vector>

template<typename T>
class DisplacementPacker
{
	MortonOrderer mo;

public:
	void FillImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, std::vector<Eigen::Vector3d>& data,
		T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val);

	void RetrieveImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, std::vector<Eigen::Vector3d>& output_data,
		T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val);


};

template<typename T>
inline void DisplacementPacker<T>::FillImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, std::vector<Eigen::Vector3d>& data,
	T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val)
{
	Eigen::Vector3d span_data = max_data_val - min_data_val;

	double span_img = (double)max_img_val - (double)min_img_val;

	size_t image_coords[2];
	size_t morton_x;
	size_t morton_y;

	size_t block_size_sqr = block_size * block_size;

	size_t loc = 0;

	for (size_t x = 0; x < image.width(); x += block_size)
	{
		for (size_t y = 0; y < image.height(); y += block_size)
		{
			for (size_t i = 0; i < block_size_sqr; ++i, ++loc)
			{
				if (loc >= data.size())
				{
					return;
				}

				mo.GetNormalOrder(image_coords, 2, i);

				morton_y = image_coords[1] + y;
				morton_x = image_coords[0] + x;

				for (size_t c = 0; c < 3; ++c)
				{
					image(morton_x, morton_y, 0, c) = (T)(((data[loc][c] - min_data_val[c]) / span_data[c]) * span_img) + min_img_val;
				}
			}
		}
	}
}

template<typename T>
inline void DisplacementPacker<T>::RetrieveImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, std::vector<Eigen::Vector3d>& output_data, 
	T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val)
{
	Eigen::Vector3d span_data = max_data_val - min_data_val;

	double span_img = (double)max_img_val - (double)min_img_val;

	size_t image_coords[2];
	size_t morton_x;
	size_t morton_y;

	size_t block_size_sqr = block_size * block_size;

	size_t loc = 0;

	for (size_t x = 0; x < image.width(); x += block_size)
	{
		for (size_t y = 0; y < image.height(); y += block_size)
		{
			for (size_t i = 0; i < block_size_sqr; ++i, ++loc)
			{
				if (loc >= output_data.size())
				{
					return;
				}

				mo.GetNormalOrder(image_coords, 2, i);

				morton_y = image_coords[1] + y;
				morton_x = image_coords[0] + x;

				for (size_t c = 0; c < 3; ++c)
				{
					output_data[loc][c] = ((image(morton_x, morton_y, 0, c) - min_img_val) / span_img) * span_data[c] + min_data_val[c];
				}
			}
		}
	}
}