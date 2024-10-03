#include "PCA_Encoder.h"

PCA_Encoder::PCA_Encoder(size_t rows, size_t cols)
{
	pca_mat.setZero(rows, cols);
}

PCA_Encoder::PCA_Encoder()
{
}

PCA_Encoder::~PCA_Encoder()
{
	CleanUpData();
}

void PCA_Encoder::Initialize(size_t rows, size_t cols)
{
	CleanUpData();

	pca_mat.setZero(rows, cols);
}

void PCA_Encoder::AddData(size_t row_num, double* data)
{
	for (size_t i = 0; i < pca_mat.cols(); ++i)
	{
		pca_mat(row_num, i) = data[i];
	}
}

void PCA_Encoder::AddData(size_t row_num, unsigned char* data)
{
	for (size_t i = 0; i < pca_mat.cols(); ++i)
	{
		pca_mat(row_num, i) = data[i];
	}
}

void PCA_Encoder::AddDataBlock(size_t row_start, size_t column_start, size_t row_end, size_t column_end, double* data)
{
	size_t x_span = row_end - row_start;
	//size_t y_span = column_end - column_start;

	for (size_t x = 0, c = column_start; c < column_end; ++c, ++x)
	{
		for (size_t y = 0, r = row_start; r < row_end; ++r, ++y)
		{
			pca_mat(r, c) = data[x_span * x + y];
			//pca_mat(r, c) = data[y_span * y + x];
		}
	}
}

void PCA_Encoder::AddDataBlock(size_t row_start, size_t column_start, size_t row_end, size_t column_end, unsigned char* data)
{
	size_t x_span = row_end - row_start;
	//size_t y_span = column_end - column_start;

	for (size_t x = 0, c = column_start; c < column_end; ++c, ++x)
	{
		for (size_t y = 0, r = row_start; r < row_end; ++r, ++y)
		{
			pca_mat(r, c) = data[x_span * x + y];
			//pca_mat(r, c) = data[y_span * y + x];
		}
	}
}


void PCA_Encoder::ConductSVD()
{
	//CleanUpData();

	svd.compute(pca_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
	//svd.compute(pca_mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
}

double PCA_Encoder::GetCenterOfData()
{
	return pca_mat.sum() / (pca_mat.rows() * pca_mat.cols());
}

Eigen::VectorXd PCA_Encoder::GetRowAverages()
{
	Eigen::VectorXd to_return;
	to_return.resize(pca_mat.cols());

	for (int c = 0; c < pca_mat.cols(); ++c)
	{
		to_return(c) = 0;

		for (int r = 0; r < pca_mat.rows(); ++r)
		{
			to_return(c) += pca_mat(r, c);
		}
	}

	to_return /= pca_mat.cols();

	return to_return;
}

void PCA_Encoder::TransposeMatrix()
{
	pca_mat.transposeInPlace();
}

void PCA_Encoder::AddValueToData(double value)
{
	pca_mat.array() += value;
}

void PCA_Encoder::AddValuesToRows(Eigen::VectorXd values)
{
	for (int r = 0; r < pca_mat.rows(); ++r)
	{
		for (int c = 0; c < pca_mat.cols(); ++c)
		{
			pca_mat(r, c) += values(c);
		}
	}
}

void PCA_Encoder::RelaySingularValues()
{
	Eigen::VectorXd vals = svd.singularValues();

	for (size_t i = 0; i < vals.size(); ++i)
	{
		std::cout << i << ": " << vals[i] << std::endl;
	}
}

void PCA_Encoder::RefreshMatrix()
{
	pca_mat.setZero(pca_mat.rows(), pca_mat.cols());
}

void PCA_Encoder::CleanUpData()
{
	//svd.

	pca_mat = Eigen::MatrixXd();
}

Eigen::MatrixXd* PCA_Encoder::GetMat()
{
	return &pca_mat;
}

Eigen::VectorXd PCA_Encoder::GetValues()
{
	return svd.singularValues();
}

Eigen::MatrixXd PCA_Encoder::GetMatrixU()
{
	return svd.matrixU();
}

Eigen::MatrixXd PCA_Encoder::GetMatrixV()
{
	return svd.matrixV();
}

void PCA_Encoder::Recreate(Eigen::MatrixXd& U, Eigen::VectorXd& diag, Eigen::MatrixXd& V)
{
	pca_mat = U * diag.asDiagonal() * V.transpose();
}

void PCA_Encoder::RecreateWithoutTransposingV(Eigen::MatrixXd& U, Eigen::VectorXd& diag, Eigen::MatrixXd& V)
{
	pca_mat = U * diag.asDiagonal() * V;
}

void PCA_Encoder::ExtractData(size_t row_num, double* presized_data_array)
{
	for (size_t i = 0; i < pca_mat.cols(); ++i)
	{
		presized_data_array[i] = pca_mat(row_num, i);
	}
}

void PCA_Encoder::ExtractData(size_t row_num, unsigned char* presized_data_array)
{
	for (size_t i = 0; i < pca_mat.cols(); ++i)
	{
		presized_data_array[i] = std::floor(pca_mat(row_num, i) + 0.5);
	}
}

void PCA_Encoder::ExtractDataAndClamp(size_t row_num, unsigned char* presized_data_array, double min, double max)
{
	for (size_t i = 0; i < pca_mat.cols(); ++i)
	{
		presized_data_array[i] = std::clamp(pca_mat(row_num, i), min, max);
	}
}

void PCA_Encoder::ExtractDataBlock(size_t row_start, size_t column_start, size_t row_end, size_t column_end, double* presized_data_array)
{
	size_t x_span = row_end - row_start;
	//size_t y_span = column_end - column_start;

	for (size_t x = 0, c = column_start; c < column_end; ++c, ++x)
	{
		for (size_t y = 0, r = row_start; r < row_end; ++r, ++y)
		{
			presized_data_array[x_span * x + y] = pca_mat(r, c);
			//presized_data_array[y_span * y + x] = pca_mat(r, c);
		}
	}
}

void PCA_Encoder::ExtractDataBlock(size_t row_start, size_t column_start, size_t row_end, size_t column_end, unsigned char* presized_data_array)
{
	size_t x_span = row_end - row_start;
	//size_t y_span = column_end - column_start;

	for (size_t x = 0, c = column_start; c < column_end; ++c, ++x)
	{
		for (size_t y = 0, r = row_start; r < row_end; ++r, ++y)
		{
			presized_data_array[x_span * x + y] = std::floor(pca_mat(r, c) + 0.5);
			//presized_data_array[y_span * y + x] = std::floor(pca_mat(r, c) + 0.5);
		}
	}
}

void PCA_Encoder::ExtractDataBlockAndClamp(size_t row_start, size_t column_start, size_t row_end, size_t column_end, unsigned char* presized_data_array, double min, double max)
{
	size_t x_span = row_end - row_start;
	//size_t y_span = column_end - column_start;

	for (size_t x = 0, c = column_start; c < column_end; ++c, ++x)
	{
		for (size_t y = 0, r = row_start; r < row_end; ++r, ++y)
		{
			presized_data_array[x_span * x + y] = std::clamp(pca_mat(r, c), min, max);
			//presized_data_array[y_span * y + x] = std::clamp(pca_mat(r, c), min, max);
		}
	}
}
