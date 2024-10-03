#include "EigenFileSavingSuite.h"

void EigenFileSavingSuite::LoadTestMatrix(std::string filename)
{
	size_t temp_rows;
	size_t temp_cols;

	Eigen::VectorXd data_strip;

	if (!sfb.OpenReadBuffer(test_file_name))
	{
		std::cout << "COULD NOT OPEN FILE " << test_file_name << std::endl;
		return;
	}

	sfb.ReadObjectFromBuffer(temp_rows);
	sfb.ReadObjectFromBuffer(temp_cols);

	data_strip.setZero(temp_rows);
	
	for (size_t c = 0; c < temp_cols; ++c)
	{
		sfb.ReadArrayFromBuffer(data_strip.data(), temp_rows);

		std::cout << data_strip.transpose() << std::endl;
	}

	sfb.CloseReadBuffer();
}

void EigenFileSavingSuite::SaveTestMatrix(std::string filename)
{
	test_mat.setZero(rows, cols);

	size_t num = 0;

	for (size_t r = 0; r < test_mat.rows(); ++r)
	{
		for (size_t c = 0; c < test_mat.cols(); ++c, ++num)
		{
			test_mat(r, c) = num;
		}
	}

	size_t sub_block_rows = rows - 1;
	size_t sub_block_cols = cols - 1;

	if (!sfb.OpenWriteBuffer(test_file_name))
	{
		std::cout << "COULD NOT OPEN FILE " << test_file_name << std::endl;
		return;
	}

	sfb.WriteObjectToBuffer(sub_block_rows);
	sfb.WriteObjectToBuffer(sub_block_cols);

	sfb.WriteArrayToBuffer(test_mat.block(0, 0, sub_block_rows, sub_block_cols).data(), sub_block_rows * sub_block_cols);

	sfb.CloseWriteBuffer();
}

void EigenFileSavingSuite::run(int argc, char** argv)
{
	SaveTestMatrix(test_file_name);
	LoadTestMatrix(test_file_name);
}
