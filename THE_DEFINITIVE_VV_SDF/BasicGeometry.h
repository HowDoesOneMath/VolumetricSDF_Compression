#pragma once

#include <Eigen/Core>
#include <vector>

inline Eigen::Vector3d CrossProduct(Eigen::Vector3d v0, Eigen::Vector3d v1)
{
	return Eigen::Vector3d(
		v0.y() * v1.z() - v1.y() * v0.z(),
		v0.z() * v1.x() - v1.z() * v0.x(),
		v0.x() * v1.y() - v1.x() * v0.y()
		);
}

inline double SquaredDistanceToTriangle(
	Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2,
	Eigen::Vector3d& p01, Eigen::Vector3d& p12, Eigen::Vector3d& p20,
	Eigen::Vector3d& loc, Eigen::Vector3d& triangle_normal)
{
	Eigen::Vector3d pl0 = loc - p0;
	Eigen::Vector3d pl1 = loc - p1;
	Eigen::Vector3d pl2 = loc - p2;

	double dot_01_12 = p01.dot(p12);
	double dot_12_20 = p12.dot(p20);
	double dot_20_01 = p20.dot(p01);

	double squaredNorm_01 = p01.squaredNorm();
	double squaredNorm_12 = p12.squaredNorm();
	double squaredNorm_20 = p20.squaredNorm();

	double inverseSquaredNorm_01 = 1.0 / squaredNorm_01;
	double inverseSquaredNorm_12 = 1.0 / squaredNorm_12;
	double inverseSquaredNorm_20 = 1.0 / squaredNorm_20;

	double dot_01_l0 = p01.dot(pl0);
	double dot_01_l1 = p01.dot(pl1);
	double dot_12_l1 = p12.dot(pl1);
	double dot_12_l2 = p12.dot(pl2);
	double dot_20_l2 = p20.dot(pl2);
	double dot_20_l0 = p20.dot(pl0);

	bool positives[3];

	positives[0] = (dot_20_l0 - dot_01_l0 * dot_20_01 * inverseSquaredNorm_01) > 0;
	positives[1] = (dot_01_l1 - dot_12_l1 * dot_01_12 * inverseSquaredNorm_12) > 0;
	positives[2] = (dot_12_l2 - dot_20_l2 * dot_12_20 * inverseSquaredNorm_20) > 0;

	if ((positives[0] == positives[1]) && (positives[0] == positives[2]))
	{
		double pl0_dot_normal = pl0.dot(triangle_normal);
		return pl0_dot_normal * pl0_dot_normal / triangle_normal.squaredNorm();
	}

	double pl0_norm_sqr = pl0.squaredNorm();
	double pl1_norm_sqr = pl1.squaredNorm();
	double pl2_norm_sqr = pl2.squaredNorm();

	if (!positives[0])
	{
		if ((dot_01_l0 > 0) != (dot_01_l1 > 0))
		{
			return pl0_norm_sqr - dot_01_l0 * dot_01_l0 * inverseSquaredNorm_01;
		}
		else
		{
			return std::min(pl0_norm_sqr, pl1_norm_sqr);
		}
	}
	else if (!positives[1])
	{
		if ((dot_12_l1 > 0) != (dot_12_l2 > 0))
		{
			return pl1_norm_sqr - dot_12_l1 * dot_12_l1 * inverseSquaredNorm_12;
		}
		else
		{
			return std::min(pl1_norm_sqr, pl2_norm_sqr);
		}
	}
	else
	{
		if ((dot_20_l2 > 0) != (dot_20_l0 > 0))
		{
			return pl2_norm_sqr - dot_20_l2 * dot_20_l2 * inverseSquaredNorm_20;
		}
		else
		{
			return std::min(pl2_norm_sqr, pl0_norm_sqr);
		}
	}
}

inline Eigen::Vector3d VectorDistanceToTriangle(
	Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2,
	Eigen::Vector3d& p01, Eigen::Vector3d& p12, Eigen::Vector3d& p20,
	Eigen::Vector3d& loc, Eigen::Vector3d& triangle_normal)
{
	Eigen::Vector3d pl0 = loc - p0;
	Eigen::Vector3d pl1 = loc - p1;
	Eigen::Vector3d pl2 = loc - p2;

	double dot_01_12 = p01.dot(p12);
	double dot_12_20 = p12.dot(p20);
	double dot_20_01 = p20.dot(p01);

	double squaredNorm_01 = p01.squaredNorm();
	double squaredNorm_12 = p12.squaredNorm();
	double squaredNorm_20 = p20.squaredNorm();

	double inverseSquaredNorm_01 = 1.0 / squaredNorm_01;
	double inverseSquaredNorm_12 = 1.0 / squaredNorm_12;
	double inverseSquaredNorm_20 = 1.0 / squaredNorm_20;

	double dot_01_l0 = p01.dot(pl0);
	double dot_01_l1 = p01.dot(pl1);
	double dot_12_l1 = p12.dot(pl1);
	double dot_12_l2 = p12.dot(pl2);
	double dot_20_l2 = p20.dot(pl2);
	double dot_20_l0 = p20.dot(pl0);

	bool positives[3];

	positives[0] = (dot_20_l0 - dot_01_l0 * dot_20_01 * inverseSquaredNorm_01) > 0;
	positives[1] = (dot_01_l1 - dot_12_l1 * dot_01_12 * inverseSquaredNorm_12) > 0;
	positives[2] = (dot_12_l2 - dot_20_l2 * dot_12_20 * inverseSquaredNorm_20) > 0;

	if ((positives[0] == positives[1]) && (positives[0] == positives[2]))
	{
		return pl0.dot(triangle_normal) * triangle_normal / triangle_normal.squaredNorm();
	}

	double pl0_norm_sqr = pl0.squaredNorm();
	double pl1_norm_sqr = pl1.squaredNorm();
	double pl2_norm_sqr = pl2.squaredNorm();

	if (!positives[0])
	{
		if ((dot_01_l0 > 0) != (dot_01_l1 > 0))
		{
			return pl0 - pl0.dot(p01) * inverseSquaredNorm_01 * p01;
		}
		else
		{
			return (pl0_norm_sqr < pl1_norm_sqr) ? pl0 : pl1;
		}
	}
	else if (!positives[1])
	{
		if ((dot_12_l1 > 0) != (dot_12_l2 > 0))
		{
			return pl1 - pl1.dot(p12) * inverseSquaredNorm_12 * p12;
		}
		else
		{
			return (pl1_norm_sqr < pl2_norm_sqr) ? pl1 : pl2;
		}
	}
	else
	{
		if ((dot_20_l2 > 0) != (dot_20_l0 > 0))
		{
			return pl2 - pl2.dot(p20) * inverseSquaredNorm_20 * p20;
		}
		else
		{
			return (pl2_norm_sqr < pl0_norm_sqr) ? pl2 : pl0;
		}
	}
}

inline std::pair<bool, double> LinecastTriangle(Eigen::Vector3d* origin, Eigen::Vector3d* direction,
	Eigen::Vector3d* p0, Eigen::Vector3d* p1, Eigen::Vector3d* p2, Eigen::Vector3d* normal)
{
	auto to_return = std::make_pair<bool, double>(true, 0.0);

	Eigen::Vector3d p0_origin = *p0 - *origin;
	double perp_dist = normal->dot(p0_origin);
	//Eigen::Vector3d perp = *normal * perp_dist;

	to_return.second = perp_dist / direction->dot(*normal);

	Eigen::Vector3d c01_o1 = CrossProduct((*p0 - *p1), (*origin - *p1));
	Eigen::Vector3d c12_o2 = CrossProduct((*p1 - *p2), (*origin - *p2));
	Eigen::Vector3d c20_o0 = CrossProduct((*p2 - *p0), (*origin - *p0));

	double n01_o1 = direction->dot(c01_o1);
	double n12_o2 = direction->dot(c12_o2);
	double n20_o0 = direction->dot(c20_o0);

	bool n01_o1_pos = (n01_o1 > 0);
	bool n12_o2_pos = (n12_o2 > 0);
	bool n20_o0_pos = (n20_o0 > 0);

	to_return.first = (n01_o1_pos == n12_o2_pos) && (n01_o1_pos == n20_o0_pos);

	return to_return;
}

inline std::pair<bool, double> LinecastTriangle(Eigen::Vector3d *origin, Eigen::Vector3d *direction,
	Eigen::Vector3d *p0, Eigen::Vector3d *p1, Eigen::Vector3d *p2)
{
	Eigen::Vector3d plane_normal = CrossProduct(*p1 - *p0, *p2 - *p0).normalized();

	return LinecastTriangle(origin, direction, p0, p1, p2, &plane_normal);
}

template<typename T>
inline void GetBarycentricCoordinatesOfTriangle(T &loc, T &p0, T &p1, T &p2, Eigen::Vector3d &output_coords)
{
	T v0 = p1 - p0, v1 = p2 - p0, v2 = loc - p0;

	double d00 = v0.dot(v0);
	double d01 = v0.dot(v1);
	double d11 = v1.dot(v1);
	double d20 = v2.dot(v0);
	double d21 = v2.dot(v1);

	double denom = d00 * d11 - d01 * d01;

	output_coords.y() = (d11 * d20 - d01 * d21) / denom;
	output_coords.z() = (d00 * d21 - d01 * d20) / denom;
	output_coords.x() = 1.0f - output_coords.y() - output_coords.z();
}

inline bool PointInsideTriangle(Eigen::Vector2d& loc, Eigen::Vector2d& p0, Eigen::Vector2d& p1, Eigen::Vector2d& p2)
{
	Eigen::Vector2d pl0 = loc - p0;
	Eigen::Vector2d pl1 = loc - p1;
	Eigen::Vector2d pl2 = loc - p2;

	Eigen::Vector2d p01 = p0 - p1;
	Eigen::Vector2d p12 = p1 - p2;
	Eigen::Vector2d p20 = p2 - p0;

	double c0 = pl0.x() * p20.y() - pl0.y() * p20.x();
	double c1 = pl1.x() * p01.y() - pl1.y() * p01.x();
	double c2 = pl2.x() * p12.y() - pl2.y() * p12.x();

	bool g0 = c0 > 0;
	bool g1 = c1 > 0;
	bool g2 = c2 > 0;

	return (c0 != 0) && (c1 != 0) && (c2 != 0) && (g0 == g1) && (g0 == g2);
}

inline bool PointInsideTriangle(Eigen::Vector3d &barycentric_coords)
{
	return (
		barycentric_coords.x() >= 0.0 && barycentric_coords.x() <= 1.0 &&
		barycentric_coords.y() >= 0.0 && barycentric_coords.y() <= 1.0 &&
		barycentric_coords.z() >= 0.0 && barycentric_coords.z() <= 1.0
		);
	//if (
	//	barycentric_coords.x() < 0.0 || barycentric_coords.x() > 1.0 ||
	//	barycentric_coords.y() < 0.0 || barycentric_coords.y() > 1.0 ||
	//	barycentric_coords.z() < 0.0 || barycentric_coords.z() > 1.0
	//	)
	//{
	//	return false;
	//}
	//
	//return true;
}

template<typename T>
inline void SwizzleRasterizedBlock(Eigen::Vector3i &dimensions, T* data, Eigen::Vector3i swizzled_axes)
{
	size_t span_y = dimensions[2];
	size_t span_x = dimensions[1] * dimensions[2];

	size_t new_span_z = dimensions[swizzled_axes[2]];
	size_t new_span_y = dimensions[swizzled_axes[1]] * new_span_z;
	size_t new_span_x = dimensions[swizzled_axes[0]] * new_span_y;
}

template<typename T>
inline Eigen::VectorX<T> TruncateVector(Eigen::VectorXd to_truncate, double max_original_value, double min_original_value, T max_mapped_value, T min_mapped_value)
{
	Eigen::VectorX<T> to_return(to_truncate.size());

	double original_range = max_original_value - min_original_value;
	T mapped_range = max_mapped_value - min_mapped_value;

	if (original_range == 0.0)
	{
		to_return.fill(min_mapped_value);
	}
	else
	{
		for (size_t i = 0; i < to_return.size(); ++i)
		{
			to_return(i) = mapped_range * ((to_truncate(i) - min_original_value) / original_range) + min_mapped_value;
		}
	}

	return to_return;
}

template<typename T>
inline Eigen::VectorXd UntruncateVector(Eigen::VectorX<T> truncated, double max_original_value, double min_original_value, T max_mapped_value, T min_mapped_value)
{
	Eigen::VectorXd to_return(truncated.size());

	double original_range = max_original_value - min_original_value;
	T mapped_range = max_mapped_value - min_mapped_value;

	if (mapped_range == 0)
	{
		to_return.fill(min_original_value);
	}
	else
	{
		for (size_t i = 0; i < to_return.size(); ++i)
		{
			to_return(i) = (original_range * (truncated(i) - min_mapped_value)) / mapped_range + min_original_value;
		}
	}

	return to_return;
}

template<typename T>
inline Eigen::MatrixX<T> TruncateMatrix(Eigen::MatrixXd to_truncate, double max_original_value, double min_original_value, T max_mapped_value, T min_mapped_value)
{
	Eigen::MatrixX<T> to_return(to_truncate.rows(), to_truncate.cols());

	double original_range = max_original_value - min_original_value;
	T mapped_range = max_mapped_value - min_mapped_value;

	if (original_range == 0.0)
	{
		to_return.fill(min_mapped_value);
	}
	else
	{
		for (size_t r = 0; r < to_return.rows(); ++r)
		{
			for (size_t c = 0; c < to_return.cols(); ++c)
			{
				to_return(r, c) = mapped_range * ((to_truncate(r, c) - min_original_value) / original_range) + min_mapped_value;
			}
		}
	}

	return to_return;
}

template<typename T>
inline Eigen::MatrixXd UntruncateMatrix(Eigen::MatrixX<T> truncated, double max_original_value, double min_original_value, T max_mapped_value, T min_mapped_value)
{
	Eigen::MatrixXd to_return(truncated.rows(), truncated.cols());

	double original_range = max_original_value - min_original_value;
	T mapped_range = max_mapped_value - min_mapped_value;

	if (mapped_range == 0)
	{
		to_return.fill(min_original_value);
	}
	else
	{
		for (size_t r = 0; r < to_return.rows(); ++r)
		{
			for (size_t c = 0; c < to_return.cols(); ++c)
			{
				to_return(r, c) = (original_range * (truncated(r, c) - min_mapped_value)) / mapped_range + min_original_value;
			}
		}
	}

	return to_return;
}