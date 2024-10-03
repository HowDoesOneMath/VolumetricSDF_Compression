#pragma once

#include <Eigen/Core>

struct VV_SparseTSDF_Stats
{
	double value;
	Eigen::Vector3i verts_on_edge = Eigen::Vector3i(-1, -1, -1);
};