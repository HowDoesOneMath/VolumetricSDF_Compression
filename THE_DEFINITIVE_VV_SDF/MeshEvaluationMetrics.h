#pragma once

#include "PointCloudGenerator.h"
#include "VV_Mesh.h"

class MeshEvaluationMetrics
{
	PointCloudGenerator pcg;

public:
	std::pair<double, double> OneWayPointToPoint(VV_Mesh& v1, VV_Mesh& v2,
		std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2);

	std::pair<double, double> GetPointToPointMetric(VV_Mesh& v1, VV_Mesh& v2,
		std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2);


};