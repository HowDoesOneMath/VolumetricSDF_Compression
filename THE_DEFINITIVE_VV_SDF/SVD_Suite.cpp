#include "SVD_Suite.h"

void SVD_Suite::RandomizeMatrixEigen(Eigen::MatrixXd& input_matrix)
{
	for (size_t r = 0; r < input_matrix.rows(); ++r)
	{
		for (size_t c = 0; c < input_matrix.cols(); ++c)
		{
			input_matrix(r, c) = rw.InRange(0, 64);
		}
	}
}

void SVD_Suite::CopyEigenToGSL(Eigen::MatrixXd& eigen_mat, gsl_matrix* gsl_mat)
{
	memcpy(gsl_mat->data, eigen_mat.data(), (eigen_mat.rows() * eigen_mat.cols()) * sizeof(double));
}

void SVD_Suite::CopyGSLtoGSL(gsl_matrix* src, gsl_matrix* dst)
{
	memcpy(dst->data, src->data, (src->size1 * src->size2) * sizeof(double));
}

void SVD_Suite::CompareJacobiToBDC(Eigen::Vector2i matrix_dims)
{
	Eigen::MatrixXd* chosen_mat;

	Eigen::MatrixXd pca_mat_1;
	Eigen::MatrixXd pca_mat_2;
	
	pca_mat_1.setZero(matrix_dims.x(), matrix_dims.y());
	pca_mat_2.setZero(matrix_dims.x(), matrix_dims.y());

	RandomizeMatrixEigen(pca_mat_1);
	RandomizeMatrixEigen(pca_mat_2);

	Eigen::JacobiSVD<Eigen::MatrixXd> jacobi_SVD;
	Eigen::BDCSVD<Eigen::MatrixXd> BDC_SVD;

	times_Jacobi.resize(iterations);
	times_BDC.resize(iterations);

	std::chrono::steady_clock::time_point time_stamp;


	std::cout << "Starting..." << std::endl;

	for (size_t i = 0; i < iterations; ++i)
	{
		chosen_mat = (i % 2 == 0 ? &pca_mat_1 : &pca_mat_2);

		std::cout << "BDC test " << i << "..." << std::endl;

		time_stamp = std::chrono::high_resolution_clock::now();
		BDC_SVD.compute(*chosen_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
		times_BDC[i] = (std::chrono::high_resolution_clock::now() - time_stamp).count();
	}

	for (size_t i = 0; i < iterations; ++i)
	{
		chosen_mat = (i % 2 == 0 ? &pca_mat_1 : &pca_mat_2);

		std::cout << "Jacobi test " << i << "..." << std::endl;

		time_stamp = std::chrono::high_resolution_clock::now();
		jacobi_SVD.compute(*chosen_mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
		times_Jacobi[i] = (std::chrono::high_resolution_clock::now() - time_stamp).count();
	}
}

bool SVD_Suite::CompareBDCToGSL(Eigen::Vector2i matrix_dims)
{
	Eigen::MatrixXd* chosen_mat_eigen;

	Eigen::MatrixXd eigen_mat_1;
	Eigen::MatrixXd eigen_mat_2;

	eigen_mat_1.setZero(matrix_dims.x(), matrix_dims.y());
	eigen_mat_2.setZero(matrix_dims.x(), matrix_dims.y());

	RandomizeMatrixEigen(eigen_mat_1);
	RandomizeMatrixEigen(eigen_mat_2);

	Eigen::BDCSVD<Eigen::MatrixXd> BDC_SVD;


	gsl_matrix* chosen_mat_gsl;

	gsl_matrix* gsl_mat_1 = gsl_matrix_calloc(matrix_dims.x(), matrix_dims.y());
	gsl_matrix* gsl_mat_2 = gsl_matrix_calloc(matrix_dims.x(), matrix_dims.y());

	gsl_matrix* output_U = gsl_matrix_calloc(matrix_dims.x(), matrix_dims.y());
	gsl_matrix* output_V = gsl_matrix_calloc(matrix_dims.x(), matrix_dims.y());
	gsl_vector* output_S = gsl_vector_calloc(matrix_dims.y());
	gsl_vector* working_vector = gsl_vector_calloc(matrix_dims.y());

	CopyEigenToGSL(eigen_mat_1, gsl_mat_1);
	CopyEigenToGSL(eigen_mat_2, gsl_mat_2);



	times_GSL.resize(iterations);
	times_BDC.resize(iterations);

	size_t gsl_error_code;

	std::chrono::steady_clock::time_point time_stamp;


	std::cout << "Starting..." << std::endl;

	for (size_t i = 0; i < iterations; ++i)
	{
		chosen_mat_eigen = (i % 2 == 0 ? &eigen_mat_1 : &eigen_mat_2);

		std::cout << "BDC test " << i << "..." << std::endl;

		time_stamp = std::chrono::high_resolution_clock::now();
		BDC_SVD.compute(*chosen_mat_eigen, Eigen::ComputeThinU | Eigen::ComputeThinV);
		times_BDC[i] = (std::chrono::high_resolution_clock::now() - time_stamp).count();
	}

	for (size_t i = 0; i < iterations; ++i)
	{
		chosen_mat_gsl = (i % 2 == 0 ? gsl_mat_1 : gsl_mat_2);

		std::cout << "GSL test " << i << "..." << std::endl;

		time_stamp = std::chrono::high_resolution_clock::now();
		CopyGSLtoGSL(chosen_mat_gsl, output_U);
		gsl_error_code = gsl_linalg_SV_decomp(output_U, output_V, output_S, working_vector);
		times_GSL[i] = (std::chrono::high_resolution_clock::now() - time_stamp).count();

		if (gsl_error_code != 0)
		{
			std::cout << "GSL_ERROR: " << gsl_error_code << std::endl;

			gsl_matrix_free(gsl_mat_1);
			gsl_matrix_free(gsl_mat_2);

			gsl_matrix_free(output_U);
			gsl_matrix_free(output_V);
			gsl_vector_free(output_S);
			gsl_vector_free(working_vector);
			return false;
		}
	}

	gsl_matrix_free(gsl_mat_1);
	gsl_matrix_free(gsl_mat_2);

	gsl_matrix_free(output_U);
	gsl_matrix_free(output_V);
	gsl_vector_free(output_S);
	gsl_vector_free(working_vector);

	return true;
}

void SVD_Suite::DebugTimes(std::vector<size_t>& to_debug)
{
	size_t total_time = 0;

	for (size_t i = 0; i < iterations; ++i)
	{
		std::cout << "\t" << i << ": " << GetPseudoDecimal(to_debug[i], seconds_to_nanoseconds) << std::endl;

		total_time += to_debug[i];
	}

	std::cout << "\tAVERAGE: " << GetPseudoDecimal(total_time, seconds_to_nanoseconds * iterations) << std::endl;
}

std::string SVD_Suite::GetPseudoDecimal(size_t input, size_t divisor)
{
	std::string to_return = std::to_string(input / divisor);

	size_t trailing = input % divisor;

	std::string decimal_str = std::to_string(((double)trailing) / divisor);

	if (decimal_str.size() > 1)
	{
		to_return += decimal_str.substr(1, decimal_str.size() - 1);
	}

	return to_return;
}

void SVD_Suite::run(int argc, char** argv)
{
	CompareJacobiToBDC(Eigen::Vector2i(SVD_size, SVD_size));
	
	std::cout << "Times BDC: " << std::endl;
	DebugTimes(times_BDC);
	std::cout << "Times Jacobi: " << std::endl;
	DebugTimes(times_Jacobi);


	//if (!CompareBDCToGSL(Eigen::Vector2i(SVD_size, SVD_size)))
	//{
	//	return;
	//}
	//
	//std::cout << "Times BDC: " << std::endl;
	//DebugTimes(times_BDC);
	//std::cout << "Times GSL: " << std::endl;
	//DebugTimes(times_GSL);
}
