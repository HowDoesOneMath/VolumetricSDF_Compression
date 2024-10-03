#pragma once

#include "TestSuite.h"

#include <Eigen/Core>
#include <iostream>

class TriangleDistanceSuite : public TestSuite
{
	Eigen::Vector3d triangle_point_1 = Eigen::Vector3d(0, 1, 0);
	Eigen::Vector3d triangle_point_2 = Eigen::Vector3d(1, -2, 0);
	Eigen::Vector3d triangle_point_3 = Eigen::Vector3d(-1, -2, 0);

	Eigen::Vector3d location_to_check = Eigen::Vector3d(-10, 0, 1);

	double CalculateDistanceSquared(Eigen::Vector3d& p0, Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& loc);
public:
	void run(int argc, char** argv);
};