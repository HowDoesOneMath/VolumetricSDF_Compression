#pragma once

#include <Eigen/Core>

class StaticDCT_3D
{
	Eigen::Vector3i dims;
	size_t vec_size;
	size_t vec_size_sqr;

	Eigen::VectorXd dct_vec;
	Eigen::VectorXd dct_section;
	Eigen::VectorXd cosine_vec_x;
	Eigen::VectorXd cosine_vec_y;
	Eigen::VectorXd cosine_vec_z;

	size_t span_y;
	size_t span_x;

	void FillCosineVec(Eigen::VectorXd& vec, size_t sqrt_len);

	void CalculateSection(Eigen::Vector3i loc);

public:
	StaticDCT_3D(Eigen::Vector3i dims);

	Eigen::VectorXd ExtractSubDCT(Eigen::Vector3i loc);

	Eigen::VectorXd ConductOnBlock(double* data);
};