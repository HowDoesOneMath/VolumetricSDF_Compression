#pragma once

#include "PointCloudGenerator.h"
#include "VV_Mesh.h"

#include "VV_CGAL_Marshaller.h"

class MeshEvaluationMetrics
{
	PointCloudGenerator pcg;

	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;

public:
	std::pair<double, double> OneWayPointToPoint(std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2);

	std::pair<double, double> GetPointToPointMetric(std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2);

	std::pair<double, double> OneWayPointToPlane(std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2);

	std::pair<double, double> GetPointToPlaneMetric(std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2);

	std::pair<double, double> OneWayHausdorffChamfer(std::vector<PointCloudGenerator::PCG_Point>& point_cloud, VV_Mesh& mesh);

	std::pair<double, double> GetHausdorffChamferDistanceMetric(VV_Mesh& v1, VV_Mesh &v2,
		std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2);
};