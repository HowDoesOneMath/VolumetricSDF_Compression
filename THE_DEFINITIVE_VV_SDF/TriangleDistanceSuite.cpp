#include "TriangleDistanceSuite.h"

double TriangleDistanceSuite::CalculateDistanceSquared(Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& loc)
{
	Eigen::Vector3d p01 = p0 - p1;
	Eigen::Vector3d p12 = p1 - p2;
	Eigen::Vector3d p20 = p2 - p0;

	Eigen::Vector3d pl0 = loc - p0;
	Eigen::Vector3d pl1 = loc - p1;
	Eigen::Vector3d pl2 = loc - p2;

	Eigen::Vector3d triangle_normal = Eigen::Vector3d(
		p01.y() * p20.z() - p01.z() * p20.y(),
		p01.z() * p20.x() - p01.x() * p20.z(),
		p01.x() * p20.y() - p01.y() * p20.x()
	);

	double dot_01_12 = p01.dot(p12);
	double dot_12_20 = p12.dot(p20);
	double dot_20_01 = p20.dot(p01);

	double squaredNorm_01 = p01.squaredNorm();
	double squaredNorm_12 = p12.squaredNorm();
	double squaredNorm_20 = p20.squaredNorm();

	double dot_01_l0 = p01.dot(pl0);
	double dot_01_l1 = p01.dot(pl1);
	double dot_12_l1 = p12.dot(pl1);
	double dot_12_l2 = p12.dot(pl2);
	double dot_20_l2 = p20.dot(pl2);
	double dot_20_l0 = p20.dot(pl0);

	bool positives[3];

	positives[0] = (dot_20_l0 - dot_01_l0 * dot_20_01 / squaredNorm_01) > 0;
	positives[1] = (dot_01_l1 - dot_12_l1 * dot_01_12 / squaredNorm_12) > 0;
	positives[2] = (dot_12_l2 - dot_20_l2 * dot_12_20 / squaredNorm_20) > 0;

	if ((positives[0] == positives[1]) && (positives[0] == positives[2]))
	{
		double pl0_dot_normal = pl0.dot(triangle_normal);
		return pl0_dot_normal * pl0_dot_normal / triangle_normal.squaredNorm();
	}

	if ((dot_01_l0 > 0) != (dot_01_l1 > 0))
	{
		return std::min(pl2.squaredNorm(), pl0.squaredNorm() - dot_01_l0 * dot_01_l0 / squaredNorm_01);
	}
	if ((dot_12_l1 > 0) != (dot_12_l2 > 0))
	{
		return std::min(pl0.squaredNorm(), pl1.squaredNorm() - dot_12_l1 * dot_12_l1 / squaredNorm_12);
	}
	if ((dot_20_l2 > 0) != (dot_20_l0 > 0))
	{
		return std::min(pl1.squaredNorm(), pl2.squaredNorm() - dot_20_l2 * dot_20_l2 / squaredNorm_20);
	}

	return std::min(std::min(pl2.squaredNorm(), pl1.squaredNorm()), pl0.squaredNorm());
}

void TriangleDistanceSuite::run(int argc, char** argv)
{
	double result = sqrt(CalculateDistanceSquared(triangle_point_1, triangle_point_2, triangle_point_3, location_to_check));

	std::cout << "Input: \n\t"
		<< triangle_point_1.transpose() << "\n\t"
		<< triangle_point_2.transpose() << "\n\t"
		<< triangle_point_3.transpose() << "\nTo Check: \n\t"
		<< location_to_check.transpose() << "\nResult: " << result << std::endl;
}
