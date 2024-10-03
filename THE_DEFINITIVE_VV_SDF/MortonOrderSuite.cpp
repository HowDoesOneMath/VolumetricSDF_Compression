#include "MortonOrderSuite.h"

void MortonOrderSuite::CreateMortonOrderTexture()
{
	size_t pixels_per[2] = { 16, 16 };
	size_t dims[2] = { 8, 8 };
	size_t coords[2];
	size_t morder_out;

	morton_image.assign(dims[0] * pixels_per[0], dims[1] * pixels_per[1], 1, 3, 0);

	//for (coords[0] = 0; coords[0] < dims[0]; ++coords[0])
	//{
	//	for (coords[1] = 0; coords[1] < dims[1]; ++coords[1])
	//	{
	//		morder_out = morder.GetMortonOrder(coords, 2) * 4 + 3;
	//
	//		for (size_t x = 0; x < pixels_per[0]; ++x)
	//		{
	//			for (size_t y = 0; y < pixels_per[1]; ++y)
	//			{
	//				morton_image(x + coords[0] * pixels_per[0], y + coords[1] * pixels_per[1], 0, 0) = morder_out;
	//				
	//			}
	//		}
	//	}
	//}

	for (size_t i = 0; i < (dims[0] * dims[1]); ++i)
	{
		morder.GetNormalOrder(coords, 2, i);

		for (size_t x = 0; x < pixels_per[0]; ++x)
		{
			for (size_t y = 0; y < pixels_per[1]; ++y)
			{
				morton_image(x + coords[0] * pixels_per[0], y + coords[1] * pixels_per[1], 0, 0) = i * 4 + 3;

			}
		}
	}

	morton_image.save_png(morton_tex_name.c_str());
}

void MortonOrderSuite::run(int argc, char** argv)
{
	CreateMortonOrderTexture();
}
