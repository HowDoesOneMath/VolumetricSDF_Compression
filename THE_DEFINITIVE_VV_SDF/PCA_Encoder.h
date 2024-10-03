#pragma once

//#include <Eigen/Core>
#include <Eigen/SVD>

#include <iostream>

/// <summary>
/// This uses Eigen::JacobiSVD despite it being slow.
/// The main reason is that BDCSVD will sometimes fail on random matrices, this is a known bug in Eigen and one that has yet to be fixed.
/// </summary>
class PCA_Encoder 
{
	Eigen::MatrixXd pca_mat;

	//Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ComputeThinU | Eigen::ComputeThinV>* svd = nullptr;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd;
	//Eigen::BDCSVD<Eigen::MatrixXd> svd;
public:
	PCA_Encoder(size_t rows, size_t cols);

	PCA_Encoder();
	
	~PCA_Encoder();

	void Initialize(size_t rows, size_t cols);

	double GetCenterOfData();
	Eigen::VectorXd GetRowAverages();

	void TransposeMatrix();

	void AddValueToData(double value);
	void AddValuesToRows(Eigen::VectorXd values);

	void AddData(size_t row_num, double* data);

	void AddData(size_t row_num, unsigned char* data);

	void AddDataBlock(size_t row_start, size_t column_start, size_t row_end, size_t column_end, double* data);

	void AddDataBlock(size_t row_start, size_t column_start, size_t row_end, size_t column_end, unsigned char* data);

	void ConductSVD();

	void RelaySingularValues();

	void RefreshMatrix();

	void CleanUpData();

	Eigen::MatrixXd* GetMat();

	Eigen::VectorXd GetValues();

	Eigen::MatrixXd GetMatrixU();
	Eigen::MatrixXd GetMatrixV();

	void Recreate(Eigen::MatrixXd &U, Eigen::VectorXd &diag, Eigen::MatrixXd &V);
	void RecreateWithoutTransposingV(Eigen::MatrixXd &U, Eigen::VectorXd &diag, Eigen::MatrixXd &V);

	void ExtractData(size_t row_num, double* presized_data_array);
	void ExtractData(size_t row_num, unsigned char* presized_data_array);
	void ExtractDataAndClamp(size_t row_num, unsigned char* presized_data_array, double min, double max);

	void ExtractDataBlock(size_t row_start, size_t column_start, size_t row_end, size_t column_end, double* presized_data_array);
	void ExtractDataBlock(size_t row_start, size_t column_start, size_t row_end, size_t column_end, unsigned char* presized_data_array);
	void ExtractDataBlockAndClamp(size_t row_start, size_t column_start, size_t row_end, size_t column_end, unsigned char* presized_data_array, double min, double max);
};