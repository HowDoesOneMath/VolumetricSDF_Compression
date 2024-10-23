#include "MeshEvaluationMetrics.h"

std::pair<double, double> MeshEvaluationMetrics::OneWayPointToPoint(std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2)
{
    auto to_return = std::make_pair<double, double>(0, 0);

    double max_dist_sqr = -DBL_MAX;

    for (size_t i = 0; i < pv1.size(); ++i)
    {
        double min_dist_sqr = DBL_MAX;

        for (size_t j = 0; j < pv2.size(); ++j)
        {
            double test_dist_sqr = (pv1[i].position - pv2[j].position).squaredNorm();

            if (test_dist_sqr < min_dist_sqr)
            {
                min_dist_sqr = test_dist_sqr;
            }
        }

        if (min_dist_sqr > max_dist_sqr)
        {
            max_dist_sqr = min_dist_sqr;
        }

        to_return.second += min_dist_sqr;
    }

    to_return.second /= pv1.size();

    to_return.first = max_dist_sqr;

    return to_return;
}

std::pair<double, double> MeshEvaluationMetrics::GetPointToPointMetric(std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2)
{
    auto to_return = std::make_pair<double, double>(0, 0);

    auto v1_to_v2 = OneWayPointToPoint(pv1, pv2);
    auto v2_to_v1 = OneWayPointToPoint(pv2, pv1);

    //to_return.second = (v1_to_v2.second * pv1.size() + v2_to_v1.second * pv2.size()) / (pv1.size() + pv2.size());
    to_return.second = v1_to_v2.second * 0.5 + v2_to_v1.second * 0.5;
    to_return.first = std::max(v1_to_v2.first, v2_to_v1.first);

    return to_return;
}

std::pair<double, double> MeshEvaluationMetrics::OneWayPointToPlane(std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2)
{
    auto to_return = std::make_pair<double, double>(0, 0);

    double max_dist_sqr = -DBL_MAX;

    for (size_t i = 0; i < pv1.size(); ++i)
    {
        size_t min_index = SIZE_MAX;
        double min_dist_sqr = DBL_MAX;

        for (size_t j = 0; j < pv2.size(); ++j)
        {
            double test_dist_sqr = (pv1[i].position - pv2[j].position).squaredNorm();

            if (test_dist_sqr < min_dist_sqr)
            {
                min_dist_sqr = test_dist_sqr;
                min_index = j;
            }
        }

        double norm_dist_sqr = abs((pv1[i].position - pv2[min_index].position).dot(pv2[min_index].normal));
        norm_dist_sqr *= norm_dist_sqr;

        if (norm_dist_sqr > max_dist_sqr)
        {
            max_dist_sqr = norm_dist_sqr;
        }

        to_return.second += norm_dist_sqr;
    }

    to_return.second /= pv1.size();

    to_return.first = max_dist_sqr;

    return to_return;
}

std::pair<double, double> MeshEvaluationMetrics::GetPointToPlaneMetric(std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2)
{
    auto to_return = std::make_pair<double, double>(0, 0);

    auto v1_to_v2 = OneWayPointToPlane(pv1, pv2);
    auto v2_to_v1 = OneWayPointToPlane(pv2, pv1);

    //to_return.second = (v1_to_v2.second * pv1.size() + v2_to_v1.second * pv2.size()) / (pv1.size() + pv2.size());
    to_return.second = v1_to_v2.second * 0.5 + v2_to_v1.second * 0.5;
    to_return.first = std::max(v1_to_v2.first, v2_to_v1.first);

    return to_return;
}

std::pair<double, double> MeshEvaluationMetrics::OneWayHausdorffChamfer(std::vector<PointCloudGenerator::PCG_Point>& point_cloud, VV_Mesh& mesh)
{
    auto to_return = std::make_pair<double, double>(0, 0);

    auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(mesh.vertices);
    auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, mesh.vertices);
    vcm.CleanMeshCGAL(*cgal_mesh);
    auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

    double max_dist = -DBL_MAX;

    for (size_t i = 0; i < point_cloud.size(); ++i)
    {
        size_t tri_index = SIZE_MAX;
        Eigen::Vector3d barycentric_coords;

        vcm.FindClosestPointCGAL(*cgal_mesh, *aabb_tree, point_cloud[i].position, tri_index, barycentric_coords);
        tri_index = (*index_remap)[tri_index];

        Eigen::Vector3d point =
            barycentric_coords.x() * mesh.vertices.elements[mesh.vertices.indices[tri_index].x()] +
            barycentric_coords.y() * mesh.vertices.elements[mesh.vertices.indices[tri_index].y()] +
            barycentric_coords.z() * mesh.vertices.elements[mesh.vertices.indices[tri_index].z()];

        double dist = sqrt(std::max((point - point_cloud[i].position).squaredNorm(), 0.0));

        if (dist > max_dist)
        {
            max_dist = dist;
        }

        to_return.second += dist;
    }

    to_return.second /= point_cloud.size();

    to_return.first = max_dist;

    return to_return;
}

std::pair<double, double> MeshEvaluationMetrics::GetHausdorffChamferDistanceMetric(VV_Mesh& v1, VV_Mesh& v2, std::vector<PointCloudGenerator::PCG_Point>& pv1, std::vector<PointCloudGenerator::PCG_Point>& pv2)
{
    auto to_return = std::make_pair<double, double>(0, 0);

    auto v1_to_v2 = OneWayHausdorffChamfer(pv1, v2);
    auto v2_to_v1 = OneWayHausdorffChamfer(pv2, v1);

    //to_return.second = (v1_to_v2.second * pv1.size() + v2_to_v1.second * pv2.size()) / (pv1.size() + pv2.size());
    to_return.second = v1_to_v2.second * 0.5 + v2_to_v1.second * 0.5;
    to_return.first = std::max(v1_to_v2.first, v2_to_v1.first);

    return to_return;
}
