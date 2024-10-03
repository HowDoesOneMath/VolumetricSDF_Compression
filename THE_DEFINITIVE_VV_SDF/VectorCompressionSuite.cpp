#include "VectorCompressionSuite.h"

void VectorCompressionSuite::TestVectorTruncation()
{
	int vector_size = 10;
	double v_min = 0;
	double v_max = 1.0;

	int trunc_min = 0;
	int trunc_max = 1000000000;

	Eigen::VectorXd vector_to_truncate(vector_size);

	for (int i = 0; i < vector_to_truncate.size(); ++i)
	{
		double t_val = (double)i / (vector_to_truncate.size() - 1);
		vector_to_truncate(i) = v_min * (1.0 - t_val) + v_max * t_val;
		vector_to_truncate(i) *= vector_to_truncate(i);
	}

	Eigen::VectorXi truncated = TruncateVector(vector_to_truncate, v_max, v_min, trunc_max, trunc_min);

	Eigen::VectorXd untruncated = UntruncateVector(truncated, v_max, v_min, trunc_max, trunc_min);

	for (int i = 0; i < vector_to_truncate.size(); ++i)
	{
		std::cout << "ORIG: " << vector_to_truncate(i) << "\tTRUNC: " << truncated(i) << "\tUNTRUNC: " << untruncated(i) << std::endl;
	}
}

void VectorCompressionSuite::TestMatrixTruncation()
{
	int matrix_rows = 4;
	int matrix_cols = 4;
	double m_min = 0;
	double m_max = 1.0;

	int trunc_min = 0;
	int trunc_max = 1000000000;

	Eigen::MatrixXd matrix_to_truncate(matrix_rows, matrix_cols);

	int unified_index = 0;
	for (int r = 0; r < matrix_to_truncate.rows(); ++r)
	{
		for (int c = 0; c < matrix_to_truncate.cols(); ++c, ++unified_index)
		{
			double t_val = (double)unified_index / (matrix_rows * matrix_cols - 1);
			matrix_to_truncate(r, c) = m_min * (1.0 - t_val) + m_max * t_val;
		}
	}

	Eigen::MatrixXi truncated = TruncateMatrix(matrix_to_truncate, m_max, m_min, trunc_max, trunc_min);

	Eigen::MatrixXd untruncated = UntruncateMatrix(truncated, m_max, m_min, trunc_max, trunc_min);

	for (int r = 0; r < matrix_to_truncate.rows(); ++r)
	{
		for (int c = 0; c < matrix_to_truncate.cols(); ++c, ++unified_index)
		{
			std::cout << "(" << r << ", " << c << ") ORIG: " << matrix_to_truncate(r, c) << "\tTRUNC: " << truncated(r, c) << "\tUNTRUNC: " << untruncated(r, c) << std::endl;
		}
	}
}

void VectorCompressionSuite::run(int argc, char** argv)
{
	//TestVectorTruncation();

	TestMatrixTruncation();
}
