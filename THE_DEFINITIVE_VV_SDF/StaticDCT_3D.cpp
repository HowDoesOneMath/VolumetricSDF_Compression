#include "StaticDCT_3D.h"

void StaticDCT_3D::FillCosineVec(Eigen::VectorXd& vec, size_t sqrt_len)
{
	double period;
	double avg;

	//This is rather inefficient when scaled up, but will likely be OK since these vectors should be small.
	//TODO: This is also wrong ATM, fix this

	for (size_t i = 0; i < vec.size(); i += sqrt_len)
	{
		avg = 0;
		for (size_t j = 0; j < sqrt_len; ++j)
		{
			period = (double)(j * i) / (double)vec.size() * EIGEN_PI;

			vec[i + j] = cos(period);
			avg += vec[i + j];
		}
	}
}

void StaticDCT_3D::CalculateSection(Eigen::Vector3i loc)
{
	Eigen::Vector3i start = loc.cwiseProduct(dims);
	Eigen::Vector3i end = start + dims;

	for (int x = start.x(); x < end.x(); ++x)
	{
		for (int y = start.y(); y < end.y(); ++y)
		{
			for (int z = start.z(); z < end.z(); ++z)
			{

			}
		}
	}
}

StaticDCT_3D::StaticDCT_3D(Eigen::Vector3i dims)
{
	this->dims = dims;

	vec_size = dims.x() * dims.y() * dims.z();
	vec_size_sqr = vec_size * vec_size;

	dct_vec.setZero(vec_size_sqr);
	dct_section.setZero(vec_size);

	cosine_vec_x.setZero(dims.x() * dims.x());
	cosine_vec_y.setZero(dims.y() * dims.y());
	cosine_vec_z.setZero(dims.z() * dims.z());

	FillCosineVec(cosine_vec_x, dims.x());
	FillCosineVec(cosine_vec_y, dims.y());
	FillCosineVec(cosine_vec_z, dims.z());

	span_y = dims.z();
	span_x = dims.y() * span_y;

	size_t loc = 0;

	for (size_t x = 0; x < dims.x(); ++x)
	{
		for (size_t y = 0; y < dims.y(); ++y)
		{
			for (size_t z = 0; z < dims.z(); ++z)
			{

			}
		}
	}
}

Eigen::VectorXd StaticDCT_3D::ExtractSubDCT(Eigen::Vector3i loc)
{
	return Eigen::VectorXd();
}

Eigen::VectorXd StaticDCT_3D::ConductOnBlock(double* data)
{
	return Eigen::VectorXd();
}
