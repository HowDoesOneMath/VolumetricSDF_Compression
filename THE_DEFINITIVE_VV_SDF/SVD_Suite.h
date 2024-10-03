#pragma once

#include "TestSuite.h"

#include <Eigen/SVD>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_linalg.h>
#include <vector>

#include "RandomWrapper.h"

#include <chrono>
#include <string>
#include <iostream>

class SVD_Suite : public TestSuite
{
	size_t SVD_size = 1024;

	size_t iterations = 100;

	size_t seconds_to_nanoseconds = 1000000000;

	std::vector<size_t> times_Jacobi;
	std::vector<size_t> times_BDC;
	std::vector<size_t> times_GSL;

	RandomWrapper rw;

	void RandomizeMatrixEigen(Eigen::MatrixXd& input_matrix);
	void CopyEigenToGSL(Eigen::MatrixXd& eigen_mat, gsl_matrix* gsl_mat);
	void CopyGSLtoGSL(gsl_matrix* src, gsl_matrix* dst);

	void CompareJacobiToBDC(Eigen::Vector2i matrix_dims);

	bool CompareBDCToGSL(Eigen::Vector2i matrix_dims);

	void DebugTimes(std::vector<size_t> &to_debug);

	std::string GetPseudoDecimal(size_t input, size_t divisor);
public:
	void run(int argc, char** argv);
};