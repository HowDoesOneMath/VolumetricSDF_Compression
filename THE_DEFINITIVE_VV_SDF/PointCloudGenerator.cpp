#include "PointCloudGenerator.h"

std::shared_ptr<std::vector<PointCloudGenerator::PCG_Point>> PointCloudGenerator::GeneratePointsFromMesh(VV_Mesh& mesh, double spacing, Eigen::Vector3d offset)
{
	auto to_return = std::make_shared<std::vector<PointCloudGenerator::PCG_Point>>();

	std::vector<std::pair<size_t, size_t>> edges;

	std::vector<Eigen::Vector3i> indices_by_edge;

	Eigen::Vector3i* tri;

	Eigen::Vector3d p0;
	Eigen::Vector3d p1;
	Eigen::Vector3d p2;

	Eigen::Vector2d p0_2d;
	Eigen::Vector2d p1_2d;
	Eigen::Vector2d p2_2d;

	Eigen::Vector3d normal_dir;
	Eigen::Vector3d abs_normal_dir;
	double max_normal_dir;
	double normal_dir_norm;

	int point_axis;
	int current_axis_to_set;
	Eigen::Vector2i remaining_axes;

	Eigen::Vector3d max_bound;
	Eigen::Vector3d min_bound;

	Eigen::Vector3i max_array;
	Eigen::Vector3i min_array;

	//Eigen::Vector3d line_origin;
	Eigen::Vector2d line_origin_2d;
	Eigen::Vector3d line_direction;

	Eigen::Vector3d barycentric_coords;

	for (size_t i = 0; i < mesh.vertices.indices.size(); ++i)
	{
		tri = &(mesh.vertices.indices[i]);

		p0 = (mesh.vertices.elements[tri->x()]) - offset;
		p1 = (mesh.vertices.elements[tri->y()]) - offset;
		p2 = (mesh.vertices.elements[tri->z()]) - offset;

		normal_dir = CrossProduct(p1 - p0, p2 - p0);
		normal_dir_norm = normal_dir.norm();

		if (normal_dir_norm <= 0)
		{
			continue;
		}

		normal_dir /= normal_dir_norm;

		abs_normal_dir = normal_dir.cwiseAbs();
		max_normal_dir = -DBL_MAX;

		for (int n = 0; n < 3; ++n)
		{
			if (max_normal_dir < abs_normal_dir[n])
			{
				max_normal_dir = abs_normal_dir[n];
				point_axis = n;
			}
		}

		line_direction = Eigen::Vector3d::Zero();
		line_direction[point_axis] = 1.0;

		current_axis_to_set = 0;
		for (int n = 0; n < 3; ++n)
		{
			if (n == point_axis)
			{
				continue;
			}

			remaining_axes[current_axis_to_set] = n;
			p0_2d[current_axis_to_set] = (p0)[n];
			p1_2d[current_axis_to_set] = (p1)[n];
			p2_2d[current_axis_to_set] = (p2)[n];

			++current_axis_to_set;
		}

		max_bound = p0.cwiseMax(p1.cwiseMax(p2)) / spacing;
		min_bound = p0.cwiseMin(p1.cwiseMin(p2)) / spacing;

		for (int n = 0; n < 3; ++n)
		{
			max_array[n] = (int)(ceil(max_bound[n]) + 0.5) + 1;
			min_array[n] = (int)(floor(min_bound[n]) + 0.5);
		}

		for (int c0 = min_array[remaining_axes[0]]; c0 < max_array[remaining_axes[0]]; ++c0)
		{
			for (int c1 = min_array[remaining_axes[1]]; c1 < max_array[remaining_axes[1]]; ++c1)
			{
				line_origin_2d[0] = c0 * spacing;
				line_origin_2d[1] = c1 * spacing;

				GetBarycentricCoordinatesOfTriangle(line_origin_2d, p0_2d, p1_2d, p2_2d, barycentric_coords);

				if (PointInsideTriangle(barycentric_coords))
				{
					to_return->push_back(PointCloudGenerator::PCG_Point());
					to_return->back().triangle = i;
					to_return->back().cast_direction = point_axis;
					to_return->back().barycentric_coords = barycentric_coords;
				}
			}
		}
	}

	return to_return;
}
