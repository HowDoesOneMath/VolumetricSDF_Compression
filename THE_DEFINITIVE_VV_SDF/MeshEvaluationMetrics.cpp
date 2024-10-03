#include "MeshEvaluationMetrics.h"

std::pair<double, double> MeshEvaluationMetrics::OneWayPointToPoint(VV_Mesh &v1, VV_Mesh &v2, 
    std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2)
{
    auto to_return = std::make_pair<double, double>(0, 0);

    Eigen::Vector3i* tri;
    Eigen::Vector3d* p0;
    Eigen::Vector3d* p1;
    Eigen::Vector3d* p2;

    Eigen::Vector3d loc1;
    Eigen::Vector3d loc2;

    double test_dist_sqr;

    double min_dist_sqr;

    double max_dist_sqr = -DBL_MAX;

    for (size_t i = 0; i < pv1.size(); ++i)
    {
        tri = &(v1.vertices.indices[pv1[i].triangle]);

        p0 = &(v1.vertices.elements[tri->x()]);
        p1 = &(v1.vertices.elements[tri->y()]);
        p2 = &(v1.vertices.elements[tri->z()]);

        loc1 =
            *p0 * pv1[i].barycentric_coords.x() +
            *p1 * pv1[i].barycentric_coords.y() +
            *p2 * pv1[i].barycentric_coords.z();

        min_dist_sqr = DBL_MAX;

        for (size_t j = 0; j < pv2.size(); ++j)
        {
            tri = &(v2.vertices.indices[pv2[i].triangle]);

            p0 = &(v2.vertices.elements[tri->x()]);
            p1 = &(v2.vertices.elements[tri->y()]);
            p2 = &(v2.vertices.elements[tri->z()]);

            loc2 =
                *p0 * pv2[i].barycentric_coords.x() +
                *p1 * pv2[i].barycentric_coords.y() +
                *p2 * pv2[i].barycentric_coords.z();

            test_dist_sqr = (loc1 - loc2).squaredNorm();

            if (test_dist_sqr < min_dist_sqr)
            {
                min_dist_sqr = test_dist_sqr;
            }
        }

        if (min_dist_sqr > max_dist_sqr)
        {
            max_dist_sqr = min_dist_sqr;
        }

        to_return.second += sqrt(min_dist_sqr);
    }

    to_return.second /= pv1.size();

    to_return.first = max_dist_sqr;

    return to_return;
}

std::pair<double, double> MeshEvaluationMetrics::GetPointToPointMetric(VV_Mesh& v1, VV_Mesh& v2,
    std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2)
{
    auto to_return = std::make_pair<double, double>(0, 0);


    return to_return;
}
