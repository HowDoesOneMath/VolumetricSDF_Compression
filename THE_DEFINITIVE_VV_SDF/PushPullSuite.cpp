#include "PushPullSuite.h"

std::shared_ptr<cimg_library::CImg<double>> PushPullSuite::ConstructTestPattern(int width, int height)
{
	auto to_return = std::make_shared<cimg_library::CImg<double>>();
	to_return->assign(width, height, 1, 3, 0);

	for (size_t w = 0; w < width; ++w)
	{
		for (size_t h = 0; h < height; ++h)
		{
			int colour_choice = (w % 2) + 2 * (h % 2);

			switch (colour_choice)
			{
			case 0:
				(*to_return)(w, h, 0, 0) = 1.0;
				break;
			case 1:
				(*to_return)(w, h, 0, 1) = 1.0;
				break;
			case 2:
				(*to_return)(w, h, 0, 2) = 1.0;
				break;
			}
		}
	}

	return to_return;
}

std::shared_ptr<cimg_library::CImg<double>> PushPullSuite::RetrieveDoubleImage(cimg_library::CImg<unsigned char>& input_image)
{
	auto to_return = std::make_shared<cimg_library::CImg<double>>();

	to_return->assign(input_image.width(), input_image.height(), 1, 4);

	for (size_t w = 0; w < input_image.width(); ++w)
	{
		for (size_t h = 0; h < input_image.height(); ++h)
		{
			(*to_return)(w, h, 0, 0) = input_image(w, h, 0, 0);
			(*to_return)(w, h, 0, 1) = input_image(w, h, 0, 1);
			(*to_return)(w, h, 0, 2) = input_image(w, h, 0, 2);

			bool has_colour = (input_image(w, h, 0, 0) > 0) || (input_image(w, h, 0, 1) > 0) || (input_image(w, h, 0, 2) > 0);

			(*to_return)(w, h, 0, 3) = has_colour;
		}
	}

	return to_return;
}

std::shared_ptr<cimg_library::CImg<unsigned char>> PushPullSuite::RetrieveByteImage(cimg_library::CImg<double>& input_image)
{
	auto to_return = std::make_shared<cimg_library::CImg<unsigned char>>();

	to_return->assign(input_image.width(), input_image.height(), 1, 3);

	for (size_t w = 0; w < input_image.width(); ++w)
	{
		for (size_t h = 0; h < input_image.height(); ++h)
		{
			(*to_return)(w, h, 0, 0) = input_image(w, h, 0, 0) * 255;
			(*to_return)(w, h, 0, 1) = input_image(w, h, 0, 1) * 255;
			(*to_return)(w, h, 0, 2) = input_image(w, h, 0, 2) * 255;
		}
	}

	return to_return;
}

void PushPullSuite::TestPushPull()
{
	test_img.assign(input_image_name.c_str());

	std::vector<bool> occupancy;
	occupancy.resize(test_img.width() * test_img.height());

	for (size_t w = 0; w < test_img.width(); ++w)
	{
		for (size_t h = 0; h < test_img.width(); ++h)
		{
			occupancy[w * test_img.height() + h] =
				(test_img(w, h, 0, 0) > 0) ||
				(test_img(w, h, 0, 1) > 0) ||
				(test_img(w, h, 0, 2) > 0);
		}
	}

	cimg_library::CImg<unsigned char> padded_img(test_img);

	int kernel_w = 5;
	double kernel_scale = 2.0;

	auto push_kernel = tr.GetGaussianKernel(kernel_w, kernel_scale, true);
	auto pull_kernel = tr.GetGaussianKernel(kernel_w, kernel_scale, false);

	//test_img = g_kernel * 255.0;
	//test_img.append(unnormalized_g_kernel * 255.0);

	tr.PadTextureWithPushPull(padded_img, occupancy, pull_kernel, push_kernel);
	test_img.append(padded_img, 'x');
	
	test_img.save_png(save_file_name.c_str());
}

void PushPullSuite::TestMul()
{
	test_img.assign(input_image_name.c_str());

	cimg_library::CImg<double> mul_img;
	cimg_library::CImg<double> test_mult_img;

	mul_img.assign(test_img.width(), test_img.height(), 1, 3, 0);

	double spacing = 0.1;

	test_mult_img.assign(test_img.width(), test_img.height(), 1, 1, 0.5);

	for (size_t w = 0; w < mul_img.width(); ++w)
	{
		for (size_t h = 0; h < mul_img.height(); ++h)
		{
			mul_img(w, h, 0, 0) = test_img(w, h, 0, 0) / 255.0;
			mul_img(w, h, 0, 1) = test_img(w, h, 0, 1) / 255.0;
			mul_img(w, h, 0, 2) = test_img(w, h, 0, 2) / 255.0;

			test_mult_img(w, h, 0, 0) = 0.5 + 0.5 * sin(w * spacing) * sin(h * spacing);
		}
	}

	mul_img.mul(test_mult_img);

	for (size_t w = 0; w < mul_img.width(); ++w)
	{
		for (size_t h = 0; h < mul_img.height(); ++h)
		{
			test_img(w, h, 0, 0) = mul_img(w, h, 0, 0) * 255.0;
			test_img(w, h, 0, 1) = mul_img(w, h, 0, 1) * 255.0;
			test_img(w, h, 0, 2) = mul_img(w, h, 0, 2) * 255.0;

			test_mult_img(w, h, 0, 0) = 0.5 + 0.5 * sin(w * spacing) * sin(h * spacing);
		}
	}

	test_img.save_png(save_file_name.c_str());
}

void PushPullSuite::TestConvolveCrop()
{
	size_t size_dims = 512;

	auto double_img = ConstructTestPattern(size_dims, size_dims);

	cimg_library::CImg<double> double_kernel;

	int kernel_w = 2;
	int kernel_h = kernel_w;

	double_kernel.assign(kernel_w, kernel_h, 1, 1, 1.0 / (kernel_h * kernel_w));

	int stride_val = 2;
	int dilation_val = 1;

	cimg_library::CImg<double> new_img;
	cimg_library::CImg<double> stride_changed_img;

	stride_changed_img.assign(*double_img);
	new_img.assign(*double_img);

	new_img.convolve(double_kernel, 0, false, 1);
	stride_changed_img.convolve(double_kernel, 0, false, 1,
		(int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
		0, 0, 0,
		(int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
		stride_val, stride_val, 1,
		dilation_val, dilation_val, 1);

	test_img = *RetrieveByteImage(*double_img);
	test_img.append(*RetrieveByteImage(new_img), 'x');
	test_img.append(*RetrieveByteImage(stride_changed_img), 'x');

	test_img.save_png(save_file_name.c_str());
}

void PushPullSuite::run(int argc, char** argv)
{
	TestPushPull();

	//TestConvolveCrop();
	
	//TestMul();
}
