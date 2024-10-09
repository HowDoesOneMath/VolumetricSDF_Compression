#include "VV_TSDF.h"

void VV_TSDF::RaycastDirection(VV_Mesh& mesh, Eigen::Vector3i& grid_dims, Eigen::Vector3d grid_corner, int raycast_dim, Eigen::Vector2i axis_dims, size_t* spans)
{
	//to_return->resize(grid_dims[axis_dims[0]]);
	//for (size_t i = 0; i < to_return->size(); ++i)
	//{
	//	(*to_return)[i].resize(grid_dims[axis_dims[1]]);
	//}

	Eigen::Vector2d corner_2d;
	corner_2d[0] = grid_corner[axis_dims[0]];
	corner_2d[1] = grid_corner[axis_dims[1]];

	Eigen::Vector3i* tri;

	Eigen::Vector3d* p0;
	Eigen::Vector3d* p1;
	Eigen::Vector3d* p2;

	Eigen::Vector2d p0_2d;
	Eigen::Vector2d p1_2d;
	Eigen::Vector2d p2_2d;

	Eigen::Vector3d normal;

	Eigen::Vector2d tri_min_2d;
	Eigen::Vector2d tri_max_2d;

	Eigen::Vector2i box_min_2d;
	Eigen::Vector2i box_max_2d;
	Eigen::Vector2i box_span_2d;

	Eigen::Vector3d raycast_origin;
	raycast_origin[raycast_dim] = grid_corner[raycast_dim];
	Eigen::Vector3d raycast_direction = Eigen::Vector3d::Zero();
	raycast_direction[raycast_dim] = 1;

	std::pair<bool, double> raycast_result;

	std::pair<size_t, std::pair<bool, double>> index_direction_position = std::make_pair(0, std::make_pair(false, 0.0));

	size_t loc;
	size_t semi_loc;
	double position_along_axis;
	double triangle_dist;

	size_t stab_count = 0;

	for (size_t i = 0; i < mesh.vertices.indices.size(); ++i)
	{
		tri = &(mesh.vertices.indices[i]);

		p0 = &(mesh.vertices.elements[tri->x()]);
		p1 = &(mesh.vertices.elements[tri->y()]);
		p2 = &(mesh.vertices.elements[tri->z()]);

		normal = CrossProduct(*p1 - *p0, *p2 - *p0).normalized();

		p0_2d[0] = (*p0)[axis_dims[0]];
		p0_2d[1] = (*p0)[axis_dims[1]];
		p1_2d[0] = (*p1)[axis_dims[0]];
		p1_2d[1] = (*p1)[axis_dims[1]];
		p2_2d[0] = (*p2)[axis_dims[0]];
		p2_2d[1] = (*p2)[axis_dims[1]];
		tri_min_2d = (p0_2d.cwiseMin(p1_2d).cwiseMin(p2_2d) - corner_2d) / gds.unit_length;
		tri_max_2d = (p0_2d.cwiseMax(p1_2d).cwiseMax(p2_2d) - corner_2d) / gds.unit_length;

		box_min_2d.x() = std::max((int)std::floor(tri_min_2d.x()), 0);
		box_min_2d.y() = std::max((int)std::floor(tri_min_2d.y()), 0);
		box_max_2d.x() = std::min((int)std::ceil(tri_max_2d.x()), grid_dims[axis_dims[0]]);
		box_max_2d.y() = std::min((int)std::ceil(tri_max_2d.y()), grid_dims[axis_dims[1]]);

		//box_span_2d = box_max_2d - box_min_2d;

		//if ((box_span_2d.x() * box_span_2d.y()) > 1)
		//{
		//	std::cout << "Hit on triangle " << i << ": " 
		//		<< box_min_2d.transpose() << " --- " 
		//		<< box_max_2d.transpose() << " --------> " 
		//		<< box_span_2d.transpose() << std::endl;
		//}

		for (int a0 = box_min_2d[0]; a0 < box_max_2d[0]; ++a0)
		{
			raycast_origin[axis_dims[0]] = a0 * gds.unit_length + corner_2d[0];

			for (int a1 = box_min_2d[1]; a1 < box_max_2d[1]; ++a1)
			{
				raycast_origin[axis_dims[1]] = a1 * gds.unit_length + corner_2d[1];

				raycast_result = LinecastTriangle(&raycast_origin, &raycast_direction, p0, p1, p2, &normal);

				if (raycast_result.first)
				{
					++stab_count;

					semi_loc = (size_t)a0 * spans[axis_dims[0]] + (size_t)a1 * spans[axis_dims[1]];

					for (int iter = 0; iter < grid_dims[raycast_dim]; ++iter)
					{
						position_along_axis = iter * gds.unit_length;// +grid_corner[raycast_dim];
						loc = (size_t)iter * spans[raycast_dim] + semi_loc;

						triangle_dist = (raycast_result.second - position_along_axis) * (1 - 2 * (normal.dot(raycast_direction) > 0));

						if (abs(triangle_dist) < abs(double_grid[loc]))
						{
							double_grid[loc] = triangle_dist;
						}
					}

					//index_direction_position.first = i;
					//index_direction_position.second.first = (normal.dot(raycast_direction) > 0);
					//index_direction_position.second.second = raycast_result.second;
					//
					//(*to_return)[a0][a1].insert(index_direction_position);
				}
			}
		}
	}
}

void VV_TSDF::GetGridDistancesCGAL(VV_Mesh& mesh)
{
	auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(mesh.vertices);
	auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, mesh.vertices);
	vcm.CleanMeshCGAL(*cgal_mesh);
	auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

#pragma omp parallel for
	for (int x = 0; x < gds.dim_x; ++x)
	{
		for (int y = 0; y < gds.dim_y; ++y)
		{
			for (int z = 0; z < gds.dim_z; ++z)
			{
				size_t t_index;
				Eigen::Vector3d b_coords;

				size_t loc = (size_t)z + (size_t)y * span_y + (size_t)x * span_x;
				Eigen::Vector3d position = grid_lower_bound + Eigen::Vector3d(x, y, z) * gds.unit_length;

				vcm.FindClosestPointCGAL(*cgal_mesh, *aabb_tree, position, t_index, b_coords);

				Eigen::Vector3i* original_triangle = &mesh.vertices.indices[(*index_remap)[t_index]];

				Eigen::Vector3d mesh_position
					= mesh.vertices.elements[original_triangle->x()] * b_coords.x()
					+ mesh.vertices.elements[original_triangle->y()] * b_coords.y()
					+ mesh.vertices.elements[original_triangle->z()] * b_coords.z();

				double_grid[loc] = std::min(double_grid[loc], (mesh_position - position).squaredNorm());
			}
		}
	}

}

void VV_TSDF::FitUnsignedTriangle(VV_Mesh* mesh, size_t triangle_index, double buffer_distance)
{
	Eigen::Vector3i ind = mesh->vertices.indices[triangle_index];

	Eigen::Vector3d *p0 = &mesh->vertices.elements[ind.x()];
	Eigen::Vector3d *p1 = &mesh->vertices.elements[ind.y()];
	Eigen::Vector3d *p2 = &mesh->vertices.elements[ind.z()];

	Eigen::Vector3d p01 = (*p0) - (*p1);
	Eigen::Vector3d p12 = (*p1) - (*p2);
	Eigen::Vector3d p20 = (*p2) - (*p0);

	Eigen::Vector3d triangle_normal = Eigen::Vector3d(
		p01.y() * p20.z() - p01.z() * p20.y(), 
		p01.z() * p20.x() - p01.x() * p20.z(), 
		p01.x() * p20.y() - p01.y() * p20.x()
	);

	if (triangle_normal.squaredNorm() <= 0)
	{
		return;
	}

	Eigen::Vector3d relative_max = Eigen::Vector3d::Ones() * -DBL_MAX;
	Eigen::Vector3d relative_min = Eigen::Vector3d::Ones() * DBL_MAX;

	for (int i = 0; i < 3; ++i)
	{
		relative_max = relative_max.cwiseMax(mesh->vertices.elements[ind[i]] - grid_lower_bound);
		relative_min = relative_min.cwiseMin(mesh->vertices.elements[ind[i]] - grid_lower_bound);
	}

	Eigen::Vector3d gridbound_max = (relative_max + Eigen::Vector3d::Ones() * buffer_distance) / gds.unit_length + Eigen::Vector3d::Ones() * 2;
	Eigen::Vector3d gridbound_min = (relative_min - Eigen::Vector3d::Ones() * buffer_distance) / gds.unit_length - Eigen::Vector3d::Ones();

	Eigen::Vector3i grid_max = Eigen::Vector3i(
		std::ceil(std::min(gridbound_max.x(), (double)gds.dim_x)),
		std::ceil(std::min(gridbound_max.y(), (double)gds.dim_y)),
		std::ceil(std::min(gridbound_max.z(), (double)gds.dim_z))
	);

	Eigen::Vector3i grid_min = Eigen::Vector3i(
		std::floor(std::max(gridbound_min.x(), (double)0)),
		std::floor(std::max(gridbound_min.y(), (double)0)),
		std::floor(std::max(gridbound_min.z(), (double)0))
	);

#pragma omp parallel for
	for (int x = grid_min.x(); x < grid_max.x(); ++x)
	{
		for (int y = grid_min.y(); y < grid_max.y(); ++y)
		{
			for (int z = grid_min.z(); z < grid_max.z(); ++z)
			{
				size_t grid_index = (size_t)x * span_x + (size_t)y * span_y + (size_t)z;

				Eigen::Vector3d point = grid_lower_bound + Eigen::Vector3d(x, y, z) * gds.unit_length;

				double test_dist_sqr = SquaredDistanceToTriangle(*p0, *p1, *p2, p01, p12, p20, point, triangle_normal);

				double_grid[grid_index] = std::min(double_grid[grid_index], test_dist_sqr);
			}
		}
	}
}

double VV_TSDF::FindNearestTrianglePointOnTemporaryGrid(VV_Mesh &mesh, Eigen::Vector3i& lower_bound, Eigen::Vector3i& upper_bound, Eigen::Vector3i& center)
{
	double to_return = DBL_MAX;

	std::vector<double> distances;
	Eigen::Vector3i bound_spans = upper_bound - lower_bound;
	distances.resize(bound_spans.x() * bound_spans.y() * bound_spans.z(), DBL_MAX);

//#pragma omp parallel for
	for (int x = lower_bound.x(); x < upper_bound.x(); ++x)
	{
		for (int y = lower_bound.y(); y < upper_bound.y(); ++y)
		{
			for (int z = lower_bound.z(); z < upper_bound.z(); ++z)
			{
				size_t array_loc = ((x - lower_bound.x()) * bound_spans.y() + (y - lower_bound.y())) * bound_spans.z() + (z - lower_bound.z());

				size_t loc = (size_t)z + (size_t)y * span_y + (size_t)x * span_x;

				if (mc_solver->inds[loc] == SIZE_MAX)
				{
					continue;
				}

				size_t span = mc_solver->mc_data[mc_solver->inds[loc]].triangle_count;

				if (span <= 0)
				{
					continue;
				}

				size_t start = mc_solver->mc_data[mc_solver->inds[loc]].triangle_indices_start;
				size_t end = start + span;

				for (size_t i = start; i < end; ++i)
				{
					Eigen::Vector3d* p0 = &mesh.vertices.elements[mesh.vertices.indices[i][0]];
					Eigen::Vector3d* p1 = &mesh.vertices.elements[mesh.vertices.indices[i][1]];
					Eigen::Vector3d* p2 = &mesh.vertices.elements[mesh.vertices.indices[i][2]];

					Eigen::Vector3d p01 = (*p0) - (*p1);
					Eigen::Vector3d p12 = (*p1) - (*p2);
					Eigen::Vector3d p20 = (*p2) - (*p0);

					Eigen::Vector3d triangle_normal = Eigen::Vector3d(
						p01.y() * p20.z() - p01.z() * p20.y(),
						p01.z() * p20.x() - p01.x() * p20.z(),
						p01.x() * p20.y() - p01.y() * p20.x()
					);

					if (triangle_normal.squaredNorm() <= 0)
					{
						continue;
					}

					Eigen::Vector3d pos = Eigen::Vector3d(center.x(), center.y(), center.z()) * gds.unit_length + grid_lower_bound;

					double test_dist = SquaredDistanceToTriangle(*p0, *p1, *p2, p01, p12, p20, pos, triangle_normal);

					bool smaller = test_dist < distances[array_loc];
					distances[array_loc] = test_dist * smaller + (1 - smaller) * distances[array_loc];
				}
			}
		}
	}

	for (auto dist : distances)
	{
		bool smaller = dist < to_return;
		to_return = dist * smaller + (1 - smaller) * to_return;
	}

	return sqrt(to_return);
}

void VV_TSDF::CullInteriorShellsInTemporaryGrid(VV_Mesh& mesh)
{
	size_t remaining_triangles = 0;

	for (size_t i = 0; i < mp.partitions.size(); ++i)
	{
		remaining_triangles += mp.partitions[i].triangle_indices.size();
	}

	//std::cout << "Remaining Triangles: " << remaining_triangles << std::endl;

	size_t vert_index;

	MarchingCubesSolver::MarchingVertexData* mvd;

	for (size_t i = 0; i < mp.partitions.size(); ++i)
	{
		if (mp.partitions[i].volume > 0)
		{
			continue;
		}

		for (size_t j = 0; j < mp.partitions[i].vertex_indices.size(); ++j)
		{
			vert_index = mp.partitions[i].vertex_indices[j];
			mvd = &(mc_solver->mc_verts[vert_index]);

			mc_solver->mc_data[mvd->parent_voxel].vert_indices_on_edges[mvd->axis] = -1;
			mc_solver->mc_data[mvd->parent_voxel].triangle_count = 0;
		}
	}

	mp.NegateInteriorShells(mesh);


	size_t starting_loc;

	//We must now deal with the scenario, however unlikely, that there is a positive volume that was fully encased in a negative volume.
	//As all negative volumes must be encased in a positive volume, and all negative volumes were removed, we test if a volume is within a positive volume.
	for (int i = 0; i < mp.partitions.size(); ++i)
	{
		if (mp.partitions[i].triangle_indices.size() <= 0)
		{
			continue;
		}

		for (int j = 0; j < mp.partitions.size(); ++j)
		{
			if (i == j) continue;

			if (!mp.PartitionOverlaps(i, j))
			{
				continue;
			}

			mvd = &(mc_solver->mc_verts[mp.partitions[j].rightmost_point]);

			starting_loc = mc_solver->mc_data[mvd->parent_voxel].grid_location + span_x;

			for (size_t loc = starting_loc; loc < mc_solver->inds.size(); loc += span_x)
			{
				size_t data_index = mc_solver->inds[loc];

				if (data_index == SIZE_MAX)
					continue;

				int edge_index = mc_solver->mc_data[data_index].vert_indices_on_edges.x();

				if (edge_index < 0)
					continue;

				if (mp.vertex_partition_map[edge_index] != i)
					continue;

				if (double_grid[loc] < double_grid[loc + span_x])
				{
					for (size_t k = 0; k < mp.partitions[j].vertex_indices.size(); ++k)
					{
						vert_index = mp.partitions[j].vertex_indices[k];
						mvd = &(mc_solver->mc_verts[vert_index]);

						mc_solver->mc_data[mvd->parent_voxel].vert_indices_on_edges[mvd->axis] = -1;
						mc_solver->mc_data[mvd->parent_voxel].triangle_count = 0;
					}

					mp.NegateElementsOfPartition(mesh, j);

					break;
				}
			}
		}
	}

	remaining_triangles = 0;

	for (size_t i = 0; i < mp.partitions.size(); ++i)
	{
		remaining_triangles += mp.partitions[i].triangle_indices.size();
	}

	//std::cout << "Remaining Triangles: " << remaining_triangles << std::endl;
}

void VV_TSDF::S_TraversalFillTemporaryGrid(VV_Mesh& mesh, int reach, double buffer_distance)
{
	std::vector<bool> to_recalculate;
	to_recalculate.resize(double_grid.size(), false);

	size_t grid_loc = 0;
	size_t previous_loc;

	size_t test_loc;
	int test_axis = 2;

	int y_direction = 0;
	int z_direction = 0;

	bool is_interior = false;

	double default_exterior = buffer_distance + gds.unit_length;

	size_t interior_count = 0;
	size_t exterior_count = 0;

	//PrintCrossSectionOfSolver(80);

	//S-shaped traveral over grid.
	//We choose != instead of < or > in these for loops as the direction is constantly changing
	for (int x = 0; x != gds.dim_x; ++x)
	{
		//modulo determined forward or backward traversal
		int y_start = y_direction * (gds.dim_y - 1); //Either 0 or size_y - 1
		int y_target = (int)gds.dim_y - y_direction * (int)(gds.dim_y + 1); //Either size_y or -1;
		int y_step = -2 * y_direction + 1; //Either +1 or -1

		//Iterate over range
		for (int y = y_start; y != y_target; y += y_step)
		{
			//Similar logic as the outer loop
			int z_start = z_direction * (gds.dim_z - 1); 
			int z_target = (int)gds.dim_z - z_direction * (int)(gds.dim_z + 1);
			int z_step = -2 * z_direction + 1;

			for (int z = z_start; z != z_target; z += z_step)
			{
				//Get the index of the grid, as well as the previous index
				previous_loc = grid_loc;
				grid_loc = (size_t)z + (size_t)y * span_y + (size_t)x * span_x;

				//The only instance in which this will happen is on the first voxel
				if (previous_loc == grid_loc)
				{
					++exterior_count;
					continue;
				}

				//Determine if the current point is interior or exterior. The first point in the grid is assumed to always be exterior.
				//is_interior = CheckIfInterior(test_loc, test_axis, is_interior, current_lesser);// , Eigen::Vector3i(x, y, z));
				is_interior = CheckIfInterior(grid_loc, previous_loc, test_axis, is_interior);

				test_axis = 2;

				if (!is_interior)
				{
					//Exterior points are given a default value - if they are outside the shell, they are outside the final representation as well.
					double_grid[grid_loc] = default_exterior;
					++exterior_count;
					continue;
				}

				to_recalculate[grid_loc] = true;

				++interior_count;
			}

			z_direction = (z_direction == 0);

			test_axis = 1;
		}

		y_direction = (y_direction == 0);

		test_axis = 0;
	}

#pragma omp parallel for
	for (int x = 0; x < gds.dim_x; ++x)
	{
		for (int y = 0; y < gds.dim_y; ++y)
		{
			for (int z = 0; z < gds.dim_z; ++z)
			{
				size_t calc_loc = (size_t)z + (size_t)y * span_y + (size_t)x * span_x;

				if (to_recalculate[calc_loc])
				{
					Eigen::Vector3i center;
					center.x() = x;
					center.y() = y;
					center.z() = z;

					Eigen::Vector3i lower_bound;
					lower_bound.x() = std::max(0, center.x() - reach);
					lower_bound.y() = std::max(0, center.y() - reach);
					lower_bound.z() = std::max(0, center.z() - reach);

					Eigen::Vector3i upper_bound;
					upper_bound.x() = std::min((int)gds.dim_x, center.x() + reach);
					upper_bound.y() = std::min((int)gds.dim_y, center.y() + reach);
					upper_bound.z() = std::min((int)gds.dim_z, center.z() + reach);

					//Calculate nearest point on shell
					double nearest = FindNearestTrianglePointOnTemporaryGrid(mesh, lower_bound, upper_bound, center);
					double_grid[calc_loc] = std::max(buffer_distance - nearest, -gds.unit_length);
				}
			}
		}
	}

	std::cout << "EXT: " << exterior_count << ", INT: " << interior_count << std::endl;

	//PrintCrossSectionOfSolver(79);
	//PrintCrossSectionOfDoubleGrid(79);
}

void VV_TSDF::S_TraversalFillTemporaryGridCGAL(VV_Mesh& mesh, double buffer_distance)
{
	size_t grid_loc = 0;
	size_t previous_loc;

	size_t test_loc;
	int test_axis = 2;

	int y_direction = 0;
	int z_direction = 0;

	std::vector<bool> to_recalculate;
	to_recalculate.resize(double_grid.size(), false);

	//S-shaped traveral over grid.
	for (int x = 0; x < gds.dim_x; ++x)
	{
		//modulo determined forward or backward traversal
		int y_start = y_direction * (gds.dim_y - 1); //Either 0 or size_y - 1
		int y_target = (int)gds.dim_y - y_direction * (int)(gds.dim_y + 1); //Either size_y or -1;
		int y_step = -2 * y_direction + 1; //Either +1 or -1

		//Iterate over range
		//We choose != instead of < or > in these for loops as the direction is constantly changing
		for (int y = y_start; y != y_target; y += y_step)
		{
			//Similar logic as the outer loop
			int z_start = z_direction * (gds.dim_z - 1);
			int z_target = (int)gds.dim_z - z_direction * (int)(gds.dim_z + 1);
			int z_step = -2 * z_direction + 1;

			for (int z = z_start; z != z_target; z += z_step)
			{
				//Get the index of the grid, as well as the previous index
				previous_loc = grid_loc;
				grid_loc = (size_t)z + (size_t)y * span_y + (size_t)x * span_x;

				//The only instance in which this will happen is on the first voxel
				if (previous_loc == grid_loc)
				{
					continue;
				}

				//Determine if the current point is interior or exterior. The first point in the grid is assumed to always be exterior.
				//is_interior = CheckIfInterior(test_loc, test_axis, is_interior, current_lesser);// , Eigen::Vector3i(x, y, z));
				bool is_interior = CheckIfInterior(grid_loc, previous_loc, test_axis, is_interior);

				test_axis = 2;

				if (!is_interior)
				{
					//Exterior points are given a default value - if they are outside the shell, they are outside the final representation as well.
					double_grid[grid_loc] = DBL_MAX;
					continue;
				}

				to_recalculate[grid_loc] = true;
			}

			z_direction = (z_direction == 0);

			test_axis = 1;
		}

		y_direction = (y_direction == 0);

		test_axis = 0;
	}

	auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(mesh.vertices);
	auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, mesh.vertices);
	vcm.CleanMeshCGAL(*cgal_mesh);
	auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

#pragma omp parallel for
	for (int x = 0; x < gds.dim_x; ++x)
	{
		for (int y = 0; y < gds.dim_y; ++y)
		{
			for (int z = 0; z < gds.dim_z; ++z)
			{
				size_t calc_loc = (size_t)z + (size_t)y * span_y + (size_t)x * span_x;

				if (to_recalculate[calc_loc])
				{
					size_t t_index;
					Eigen::Vector3d b_coords;

					Eigen::Vector3d position = grid_lower_bound + Eigen::Vector3d(x, y, z) * gds.unit_length;

					vcm.FindClosestPointCGAL(*cgal_mesh, *aabb_tree, position, t_index, b_coords);

					Eigen::Vector3i* original_triangle = &mesh.vertices.indices[(*index_remap)[t_index]];

					Eigen::Vector3d mesh_position
						= mesh.vertices.elements[original_triangle->x()] * b_coords.x()
						+ mesh.vertices.elements[original_triangle->y()] * b_coords.y()
						+ mesh.vertices.elements[original_triangle->z()] * b_coords.z();

					double dist = (mesh_position - position).norm();
					double_grid[calc_loc] = std::max(buffer_distance - dist, -gds.unit_length);
				}
			}
		}
	}
}

bool VV_TSDF::CheckIfInterior(size_t test_loc, size_t previous_loc, int axis, bool currently_interior) //, bool evaluating_self)//, Eigen::Vector3i grid_coords)
{
	size_t min_loc = (test_loc < previous_loc) ? test_loc : previous_loc;
	size_t data_ind = mc_solver->inds[min_loc];
	if (data_ind == SIZE_MAX)
	{
		return currently_interior;
	}

	int vert_ind = mc_solver->mc_data[data_ind].vert_indices_on_edges[axis];

	if (vert_ind < 0)
	{
		return currently_interior;
	}

	return !currently_interior;
}

void VV_TSDF::RecalculateBounds()
{
	grid_lower_bound.x() = - 0.5 * (gds.dim_x - 1) * gds.unit_length + gds.center_x;
	grid_lower_bound.y() = - 0.5 * (gds.dim_y - 1) * gds.unit_length + gds.center_y;
	grid_lower_bound.z() = - 0.5 * (gds.dim_z - 1) * gds.unit_length + gds.center_z;

	grid_upper_bound.x() = 0.5 * (gds.dim_x - 1) * gds.unit_length + gds.center_x;
	grid_upper_bound.y() = 0.5 * (gds.dim_y - 1) * gds.unit_length + gds.center_y;
	grid_upper_bound.z() = 0.5 * (gds.dim_z - 1) * gds.unit_length + gds.center_z;

	span_x = gds.dim_y * gds.dim_z;
	span_y = gds.dim_z;
}


void VV_TSDF::ClearGrid()
{
	if (!initialized)
	{
		return;
	}

	gds.Clear();

	grid_lower_bound = Eigen::Vector3d(0, 0, 0);
	grid_upper_bound = Eigen::Vector3d(0, 0, 0);

	span_x = 0;
	span_y = 0;

	double_grid.clear();
	quantized_grid.clear();

	mp.ClearPartitions();

	initialized = false;
}

void VV_TSDF::RefreshGrid()
{
	if (!initialized)
	{
		return;
	}

	std::fill(quantized_grid.begin(), quantized_grid.end(), 0);

	if (double_grid.size() > 0)
	{
		std::fill(double_grid.begin(), double_grid.end(), 0);
	}
}

bool VV_TSDF::InitializeGrid(GridDataStruct &gds)
{
	if (initialized)
	{
		return false;
	}

	this->gds = gds;

	mc_solver = std::make_shared<MarchingCubesSolver>(gds);

	quantized_grid.resize(gds.dim_x * gds.dim_y * gds.dim_z);

	RecalculateBounds();

	initialized = true;

	return true;
}

void VV_TSDF::QuantizeGrid()
{
	if (!update_quantized)
	{
		return;
	}

	//double shift = 0.5 * gds.unit_length;
	double shift = 0.5;

	double isosurface_threshold = (double)(max_c - min_c) * 0.5;

	for (size_t i = 0; i < quantized_grid.size(); ++i)
	{
		double clamped_scaled = (std::min(gds.unit_length, double_grid[i]) + gds.unit_length) * isosurface_threshold;

		unsigned char val = (unsigned char)std::floor(clamped_scaled / gds.unit_length + shift);

		quantized_grid[i] = std::clamp(val, min_c, max_c);
	}

	update_quantized = false;
}

void VV_TSDF::ClampUnquantizedGrid()
{
	for (size_t i = 0; i < double_grid.size(); ++i)
	{
		double_grid[i] = std::clamp(double_grid[i], -gds.unit_length, gds.unit_length);
	}
}

void VV_TSDF::SubtractFromGrid(unsigned char* data_array)
{
	for (size_t i = 0; i < quantized_grid.size(); ++i)
	{
		quantized_grid[i] -= data_array[i];
	}
}

void VV_TSDF::AddToGrid(unsigned char* data_array)
{
	for (size_t i = 0; i < quantized_grid.size(); ++i)
	{
		quantized_grid[i] += data_array[i];
	}
}

bool VV_TSDF::MakeDebuggingSphere(Eigen::Vector3d center, double radius, std::string filename)
{
	double_grid.resize(quantized_grid.size());

	Eigen::Vector3d position;

	size_t loc = 0;

	for (size_t x = 0; x < gds.dim_x; ++x)
	{
		for (size_t y = 0; y < gds.dim_y; ++y)
		{
			for (size_t z = 0; z < gds.dim_z; ++z, ++loc)
			{
				position = Eigen::Vector3d(x, y, z) * gds.unit_length + grid_lower_bound;

				double_grid[loc] = ((position - center).norm() - radius) / gds.unit_length;
			}
		}
	}

	std::vector<MarchingCubesSolver::MarchingVoxelData> voxel_data;
	std::vector<MarchingCubesSolver::MarchingVertexData> vertex_data;

	size_t uv_size = 1024;

	auto mesh = mc_solver->ExtractMeshFromGrid<double>(double_grid.data(), 0);// , Eigen::Vector2i(uv_size, uv_size), 0.126);
	mc_solver->ResetIndices();

	double_grid.clear();

	return mesh->WriteOBJ(filename);
}

std::shared_ptr<VV_Mesh> VV_TSDF::CastMeshUnsignedDistance(VV_Mesh* mesh, double buffer_distance)
{
	double sqr_buffer = buffer_distance * buffer_distance;
	double buffer_plus_relaxation = buffer_distance + gds.unit_length;

	if (double_grid.size() > 0)
	{
		double_grid.clear();
	}

	double_grid.resize(quantized_grid.size(), buffer_plus_relaxation * buffer_plus_relaxation);

	for (size_t i = 0; i < mesh->vertices.indices.size(); ++i)
	{
		FitUnsignedTriangle(mesh, i, buffer_distance);
	}

	auto expanded_mesh = mc_solver->ExtractIsosurfaceMeshFromGridWithSquaredValues(double_grid.data(), buffer_distance);

	mp.CreateUnionFindPartitions((*expanded_mesh));

	CullInteriorShellsInTemporaryGrid(*expanded_mesh);

	S_TraversalFillTemporaryGrid(*expanded_mesh, buffer_plus_relaxation / gds.unit_length, buffer_distance);

	update_quantized = true;

	QuantizeGrid();

	//expanded_mesh = mc_solver->ExtractMeshFromGrid(grid_temporary.data(), 0);

	mc_solver->ResetIndices();

	//double_grid.clear();

	return expanded_mesh;
}

std::shared_ptr<VV_Mesh> VV_TSDF::CastMeshUnsignedDistanceSampleAll(VV_Mesh* mesh, double buffer_distance)
{
	double sqr_buffer = buffer_distance * buffer_distance;
	double buffer_plus_relaxation = buffer_distance + gds.unit_length;

	if (double_grid.size() > 0)
	{
		double_grid.clear();
	}

	double_grid.resize(quantized_grid.size(), DBL_MAX);

	GetGridDistancesCGAL(*mesh);

	auto expanded_mesh = mc_solver->ExtractIsosurfaceMeshFromGridWithSquaredValues(double_grid.data(), buffer_distance);

	mp.CreateUnionFindPartitions((*expanded_mesh));

	CullInteriorShellsInTemporaryGrid(*expanded_mesh);

	//S_TraversalFillTemporaryGrid(*expanded_mesh, buffer_plus_relaxation / gds.unit_length, buffer_distance);
	S_TraversalFillTemporaryGridCGAL(*expanded_mesh, buffer_distance);

	update_quantized = true;
	QuantizeGrid();

	mc_solver->ResetIndices();

	return expanded_mesh;
}

void VV_TSDF::CastMeshNaiveRaycast(VV_Mesh &mesh)
{
	if (double_grid.size() > 0)
	{
		double_grid.clear();
	}

	double_grid.resize(quantized_grid.size(), DBL_MAX);

	Eigen::Vector3i grid_dims = Eigen::Vector3i(gds.dim_x, gds.dim_y, gds.dim_z);

	size_t spans[3];

	spans[0] = span_x;
	spans[1] = span_y;
	spans[2] = 1;

	RaycastDirection(mesh, grid_dims, grid_lower_bound, 0, Eigen::Vector2i(1, 2), spans);
	RaycastDirection(mesh, grid_dims, grid_lower_bound, 1, Eigen::Vector2i(2, 0), spans);
	RaycastDirection(mesh, grid_dims, grid_lower_bound, 2, Eigen::Vector2i(0, 1), spans);

	ClampUnquantizedGrid();

	update_quantized = true;
	QuantizeGrid();
}


std::shared_ptr<VV_Mesh> VV_TSDF::ExtractMeshQuantized()//Eigen::Vector2i uv_partition_count, double uv_buffer)
{
	auto mesh = mc_solver->ExtractMeshFromGrid(quantized_grid.data(), ((double)max_c + (double)min_c) * 0.5);// , uv_partition_count, uv_buffer);

	return mesh;
}

std::shared_ptr<std::vector<size_t>> VV_TSDF::ExtractTriangleGroupsFromBlock(Eigen::Vector3i block_start, Eigen::Vector3i block_end)
{
	auto to_return = std::make_shared<std::vector<size_t>>();

	size_t mc_ind;
	size_t tri_start;
	size_t tri_end;

	for (int x = block_start.x(); x < block_end.x(); ++x)
	{
		for (int y = block_start.y(); y < block_end.y(); ++y)
		{
			for (int z = block_start.z(); z < block_end.z(); ++z)
			{
				size_t loc = x * span_x + y * span_y + z;
				mc_ind = mc_solver->inds[loc];

				if (mc_ind == SIZE_MAX) continue;

				tri_start = mc_solver->mc_data[mc_ind].triangle_indices_start;
				tri_end = (size_t)mc_solver->mc_data[mc_ind].triangle_count + tri_start;

				for (size_t t = tri_start; t < tri_end; ++t)
				{
					to_return->push_back(t);
				}
			}
		}
	}

	return to_return;
}

std::shared_ptr<std::vector<std::vector<size_t>>> VV_TSDF::ExtractTriangleGroupsByBlock(VV_Mesh& mesh, Eigen::Vector3i block_size)
{
	auto to_return = std::make_shared<std::vector<std::vector<size_t>>>();

	size_t total_block_size = block_size.x() * block_size.y() * block_size.z();
	size_t block_count = quantized_grid.size() / total_block_size;

	to_return->resize(block_count);
	size_t blocks_x = gds.dim_x / block_size.x();
	size_t blocks_y = gds.dim_y / block_size.y();
	size_t blocks_z = gds.dim_z / block_size.z();

#pragma omp parallel for
	for (int x = 0; x < blocks_x; ++x)
	{
		for (int y = 0; y < blocks_y; ++y)
		{
			for (int z = 0; z < blocks_z; ++z)//, ++current_triangle_block)
			{
				size_t current_triangle_block = ((size_t)x * blocks_y + (size_t)y) * blocks_z + (size_t)z;

				Eigen::Vector3i block_start = Eigen::Vector3i(x, y, z).cwiseProduct(block_size);
				Eigen::Vector3i block_end = block_start + block_size;

				auto new_groups = ExtractTriangleGroupsFromBlock(block_start, block_end);

				(*to_return)[current_triangle_block] = (*new_groups);
			}
		}
	}

	return to_return;
}

size_t VV_TSDF::CastGridDataStruct(VV_SaveFileBuffer& buffer)
{
	if (!initialized)
	{
		return 0;
	}

	return buffer.WriteObjectToBuffer(gds);
}

size_t VV_TSDF::RetrieveGridDataStruct(VV_SaveFileBuffer& buffer)
{
	if (initialized)
	{
		return 0;
	}

	size_t to_return = buffer.ReadObjectFromBuffer(gds);
	quantized_grid.resize(gds.dim_x * gds.dim_y * gds.dim_z);

	RecalculateBounds();

	initialized = true;

	return to_return;
}

void VV_TSDF::SmoothUnquantizedGrid()
{
	std::vector<bool> is_isosurface_any;
	is_isosurface_any.resize(double_grid.size(), false);
	std::vector<bool> is_isosurface_x;
	is_isosurface_x.resize(double_grid.size(), false);
	std::vector<bool> is_isosurface_y;
	is_isosurface_y.resize(double_grid.size(), false);
	std::vector<bool> is_isosurface_z;
	is_isosurface_z.resize(double_grid.size(), false);

	double grid_val;

	size_t grid_loc = 0;
	for (size_t x = 0; x < gds.dim_x; ++x) {
		for (size_t y = 0; y < gds.dim_y; ++y) {
			for (size_t z = 0; z < gds.dim_z; ++z) {
				grid_val = double_grid[grid_loc];

				if (x > 0 && ((double_grid[grid_loc - span_x] > 0) != (grid_val > 0))) {
					is_isosurface_x[grid_loc] = true;
					is_isosurface_any[grid_loc] = true;
				}
				else if (x < (gds.dim_x - 1) && ((double_grid[grid_loc + span_x] > 0) != (grid_val > 0))) {
					is_isosurface_x[grid_loc] = true;
					is_isosurface_any[grid_loc] = true;
				}
				if (y > 0 && ((double_grid[grid_loc - span_y] > 0) != (grid_val > 0))) {
					is_isosurface_y[grid_loc] = true;
					is_isosurface_any[grid_loc] = true;
				}
				else if (y < (gds.dim_y - 1) && ((double_grid[grid_loc + span_y] > 0) != (grid_val > 0))) {
					is_isosurface_y[grid_loc] = true;
					is_isosurface_any[grid_loc] = true;
				}
				if (z > 0 && ((double_grid[grid_loc - 1] > 0) != (grid_val > 0))) {
					is_isosurface_z[grid_loc] = true;
					is_isosurface_any[grid_loc] = true;
				}
				else if (z < (gds.dim_z - 1) && ((double_grid[grid_loc + 1] > 0) != (grid_val > 0))) {
					is_isosurface_z[grid_loc] = true;
					is_isosurface_any[grid_loc] = true;
				}

				++grid_loc;
			}
		}
	}

	for (size_t i = 0; i < is_isosurface_any.size(); ++i)
	{
		if (!is_isosurface_any[i]) double_grid[i] = DBL_MAX * (2 * (double_grid[i] >= 0) - 1);
	}

	double slope;
	size_t delta;
	size_t count;

	grid_loc = 0;
	delta = 1;
	for (int x = 0; x < gds.dim_x; ++x) {
		for (int y = 0; y < gds.dim_y; ++y) {
			count = 0;
			for (int z = 1; z < gds.dim_z; ++z) {
				grid_loc = x * span_x + y * span_y + z;
				++count;

				if (is_isosurface_z[grid_loc] && is_isosurface_z[grid_loc - delta]){
					slope = double_grid[grid_loc] - double_grid[grid_loc - delta];
					count = 0;
				}

				if (is_isosurface_any[grid_loc]) continue;

				if (abs(double_grid[grid_loc]) > abs(slope * count)) {
					double_grid[grid_loc] = slope * count;
				}
			}

			count = 0;
			for (int z = gds.dim_z - 2; z >= 0; --z) {
				grid_loc = x * span_x + y * span_y + z;
				++count;

				if (is_isosurface_z[grid_loc] && is_isosurface_z[grid_loc + delta]) {
					slope = double_grid[grid_loc] - double_grid[grid_loc + delta];
					count = 0;
				}

				if (is_isosurface_any[grid_loc]) continue;

				if (abs(double_grid[grid_loc]) > abs(slope * count)) {
					double_grid[grid_loc] = slope * count;
				}
			}
		}
	}

	grid_loc = 0;
	delta = span_y;
	for (int z = 0; z < gds.dim_z; ++z) {
		for (int x = 0; x < gds.dim_x; ++x) {
			count = 0;
			for (int y = 1; y < gds.dim_y; ++y) {
				grid_loc = x * span_x + y * span_y + z;
				++count;

				if (is_isosurface_z[grid_loc] && is_isosurface_z[grid_loc - delta]) {
					slope = double_grid[grid_loc] - double_grid[grid_loc - delta];
					count = 0;
				}

				if (is_isosurface_any[grid_loc]) continue;

				if (abs(double_grid[grid_loc]) > abs(slope * count)) {
					double_grid[grid_loc] = slope * count;
				}
			}

			count = 0;
			for (int y = gds.dim_y - 2; y >= 0; --y) {
				grid_loc = x * span_x + y * span_y + z;
				++count;

				if (is_isosurface_z[grid_loc] && is_isosurface_z[grid_loc + delta]) {
					slope = double_grid[grid_loc] - double_grid[grid_loc + delta];
					count = 0;
				}

				if (is_isosurface_any[grid_loc]) continue;

				if (abs(double_grid[grid_loc]) > abs(slope * count)) {
					double_grid[grid_loc] = slope * count;
				}
			}
		}
	}

	grid_loc = 0;
	delta = span_x;
	for (int y = 0; y < gds.dim_y; ++y) {
		for (int z = 0; z < gds.dim_z; ++z) {
			count = 0;
			for (int x = 1; x < gds.dim_x; ++x) {
				grid_loc = x * span_x + y * span_y + z;
				++count;

				if (is_isosurface_z[grid_loc] && is_isosurface_z[grid_loc - delta]) {
					slope = double_grid[grid_loc] - double_grid[grid_loc - delta];
					count = 0;
				}

				if (is_isosurface_any[grid_loc]) continue;

				if (abs(double_grid[grid_loc]) > abs(slope * count)) {
					double_grid[grid_loc] = slope * count;
				}
			}

			count = 0;
			for (int x = gds.dim_x - 2; x >= 0; --x) {
				grid_loc = x * span_x + y * span_y + z;
				++count;

				if (is_isosurface_z[grid_loc] && is_isosurface_z[grid_loc + delta]) {
					slope = double_grid[grid_loc] - double_grid[grid_loc + delta];
					count = 0;
				}

				if (is_isosurface_any[grid_loc]) continue;

				if (abs(double_grid[grid_loc]) > abs(slope * count)) {
					double_grid[grid_loc] = slope * count;
				}
			}
		}
	}
}

void VV_TSDF::PerformSmoothingOverUnquantizedBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z)
{
	size_t block_size = len_x * len_y * len_z;

	std::vector<bool> is_isosurface_x;
	is_isosurface_x.resize(block_size, false);
	std::vector<bool> is_isosurface_y;
	is_isosurface_y.resize(block_size, false);
	std::vector<bool> is_isosurface_z;
	is_isosurface_z.resize(block_size, false);

	size_t loop_begin = 0;
	size_t loop_end = block_size - 1;

	size_t loop_head = 0;

	size_t grid_loc;
	size_t test_loc;
	size_t array_loc = 0;
	double grid_val;

	for (size_t loc_x = 0, p_x = x * span_x; loc_x < len_x; ++loc_x, p_x += span_x) {
		for (size_t loc_y = 0, p_y = y * span_y; loc_y < len_y; ++loc_y, p_y += span_y) {
			for (size_t loc_z = 0, p_z = z; loc_z < len_z; ++loc_z, ++p_z, ++array_loc) {
				grid_loc = p_x + p_y + p_z;
				grid_val = double_grid[grid_loc];

				if (loc_x > 0 && ((double_grid[grid_loc - span_x] > 0) != (grid_val > 0))) {
					is_isosurface_x[array_loc] = true;
				}
				else if (loc_x < (len_x - 1) && ((double_grid[grid_loc + span_x] > 0) != (grid_val > 0))) {
					is_isosurface_x[array_loc] = true;
				}
				if (loc_y > 0 && ((double_grid[grid_loc - span_y] > 0) != (grid_val > 0))) {
					is_isosurface_y[array_loc] = true;
				}
				else if (loc_y < (len_y - 1) && ((double_grid[grid_loc + span_y] > 0) != (grid_val > 0))) {
					is_isosurface_y[array_loc] = true;
				}
				if (loc_z > 0 && ((double_grid[grid_loc - 1] > 0) != (grid_val > 0))) {
					is_isosurface_z[array_loc] = true;
				}
				else if (loc_z < (len_z - 1) && ((double_grid[grid_loc + 1] > 0) != (grid_val > 0))) {
					is_isosurface_z[array_loc] = true;
				}
			}
		}
	}

	for (size_t loc_x = 0, p_x = x * span_x; loc_x < len_x; ++loc_x, p_x += span_x) {
		for (size_t loc_y = 0, p_y = y * span_y; loc_y < len_y; ++loc_y, p_y += span_y) {
			for (size_t loc_z = 0, p_z = z; loc_z < len_z; ++loc_z, ++p_z) {
				grid_loc = p_x + p_y + p_z;
				grid_val = double_grid[grid_loc];

				array_loc = loc_z + len_z * (loc_y + len_y * loc_x);

				if (is_isosurface_x[array_loc])
				{

				}
			}
		}
	}
}

void VV_TSDF::ExtractBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char* presized_block_array)
{
	size_t grid_loc;
	size_t array_loc = 0;

	for (size_t loc_x = 0, p_x = x * span_x;  loc_x < len_x; ++loc_x, p_x += span_x)
	{
		for (size_t loc_y = 0, p_y = y * span_y; loc_y < len_y; ++loc_y, p_y += span_y, array_loc += len_z)
		{
			size_t grid_loc = p_x + p_y + z;
			size_t array_loc = (loc_x * len_y + loc_y) * len_z;

			memcpy(&(presized_block_array[array_loc]), &(quantized_grid[grid_loc]), len_z);
		}
	}
}

void VV_TSDF::InsertBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, Eigen::MatrixXd& mat, double average)
{
	//size_t grid_loc;
	//size_t array_loc = 0;

	//size_t r = 0;
	//size_t c = 0;

#pragma omp parallel for
	for (int loc_x = 0; loc_x < len_x; ++loc_x)
	{
		size_t p_x = (x + (size_t)loc_x) * span_x;

		for (int loc_y = 0; loc_y < len_y; ++loc_y)
		{
			size_t p_y = (y + (size_t)loc_y) * span_y;

			for (int loc_z = 0; loc_z < len_z; ++loc_z)
			{
				size_t grid_loc = p_x + p_y + z;
				//size_t array_loc = (loc_x * len_y + loc_y) * len_z + loc_z;

				size_t r = (((size_t)loc_x * len_y + (size_t)loc_y) * len_z + (size_t)loc_z);
				size_t c = r % mat.rows();
				r = r / mat.rows();

				quantized_grid[grid_loc] = std::clamp(mat(r, c) + average, (double)min_c, (double)max_c) + 0.1;


				//++r;
				//
				//c += (r >= mat.rows());
				//r -= r * (r >= mat.rows());
			}
		}
	}
}

void VV_TSDF::InsertBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char* presized_block_array)
{
	//size_t grid_loc;
	//size_t array_loc = 0;
 
#pragma omp parallel for
	for (int loc_x = 0; loc_x < len_x; ++loc_x)
	{
		size_t p_x = (x + (size_t)loc_x) * span_x;

		for (int loc_y = 0; loc_y < len_y; ++loc_y)//, array_loc += len_z)
		{
			size_t p_y = (y + (size_t)loc_y) * span_y;

			size_t grid_loc = p_x + p_y + z;
			size_t array_loc = (loc_x * len_y + loc_y) * len_z;

			memcpy(&(quantized_grid[grid_loc]), &(presized_block_array[array_loc]), len_z);
		}
	}
}

void VV_TSDF::ExtractBlockUnquantized(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, double* presized_block_array)
{
	size_t grid_loc;
	size_t array_loc = 0;

	for (size_t loc_x = 0, p_x = x * span_x; loc_x < len_x; ++loc_x, p_x += span_x)
	{
		for (size_t loc_y = 0, p_y = y * span_y; loc_y < len_y; ++loc_y, p_y += span_y)
		{
			for (size_t loc_z = 0, p_z = z; loc_z < len_z; ++loc_z, ++p_z, ++array_loc)
			{
				grid_loc = p_x + p_y + p_z;

				presized_block_array[array_loc] = double_grid[grid_loc] / gds.unit_length;
			}
		}
	}
}

void VV_TSDF::InsertBlockUnquantized(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, double* presized_block_array)
{
	size_t grid_loc;
	size_t array_loc = 0;

	for (size_t loc_x = 0, p_x = x * span_x; loc_x < len_x; ++loc_x, p_x += span_x)
	{
		for (size_t loc_y = 0, p_y = y * span_y; loc_y < len_y; ++loc_y, p_y += span_y)
		{
			for (size_t loc_z = 0, p_z = z; loc_z < len_z; ++loc_z, ++p_z, ++array_loc)
			{
				grid_loc = p_x + p_y + p_z;

				double_grid[grid_loc] = gds.unit_length * presized_block_array[array_loc];
			}
		}
	}

	update_quantized = true;
}

void VV_TSDF::ExtractBlockMortonOrder(size_t x, size_t y, size_t z, size_t data_size, unsigned char* presized_block_array)
{
	size_t grid_loc;
	size_t axes[3];

	for (size_t i = 0; i < data_size; ++i)
	{
		mo.GetNormalOrder(axes, 3, i);

		grid_loc = (axes[0] + x) * span_x + (axes[1] + y) * span_y + (axes[2] + z);

		presized_block_array[i] = quantized_grid[grid_loc];
	}
}

void VV_TSDF::InsertBlockMortonOrder(size_t x, size_t y, size_t z, size_t data_size, unsigned char* presized_block_array)
{
	size_t grid_loc;
	size_t axes[3];

	for (size_t i = 0; i < data_size; ++i)
	{
		mo.GetNormalOrder(axes, 3, i);

		grid_loc = (axes[0] + x) * span_x + (axes[1] + y) * span_y + (axes[2] + z);

		quantized_grid[grid_loc] = presized_block_array[i];
	}
}

void VV_TSDF::ExtractBlockS_Traversal(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char* presized_block_array)
{
	size_t grid_loc;
	size_t array_loc = 0;
	Eigen::Vector3i axes;

	int y_start;
	int y_target;
	int y_step;

	int z_start;
	int z_target;
	int z_step;

	int y_direction = 0;
	int z_direction = 0;

	Eigen::Vector3i start = Eigen::Vector3i(x, y, z);
	Eigen::Vector3i end = start + Eigen::Vector3i(len_x, len_y, len_z);

	for (axes[0] = start[0]; axes[0] != end[0]; ++(axes[0]))
	{
		y_start = y_direction * (end[1] - 1) + (1 - y_direction) * start[1];
		y_target = y_direction * (start[1] - 1) + (1 - y_direction) * end[1];
		y_step = -2 * y_direction + 1; 

		for (axes[1] = y_start; axes[1] != y_target; axes[1] += y_step)
		{
			z_start = z_direction * (end[2] - 1) + (1 - z_direction) * start[2];
			z_target = z_direction * (start[2] - 1) + (1 - z_direction) * end[2];
			z_step = -2 * z_direction + 1;

			for (axes[2] = z_start; axes[2] != z_target; axes[2] += z_step, ++array_loc)
			{
				grid_loc = (size_t)axes[2] + (size_t)axes[1] * span_y + (size_t)axes[0] * span_x;

				presized_block_array[array_loc] = quantized_grid[grid_loc];
			}

			z_direction = (z_direction == 0);
		}

		y_direction = (y_direction == 0);
	}
}

void VV_TSDF::InsertBlockS_Traversal(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char* presized_block_array)
{
	size_t grid_loc;
	size_t array_loc = 0;
	Eigen::Vector3i axes;

	int y_start;
	int y_target;
	int y_step;

	int z_start;
	int z_target;
	int z_step;

	int y_direction = 0;
	int z_direction = 0;

	Eigen::Vector3i start = Eigen::Vector3i(x, y, z);
	Eigen::Vector3i end = start + Eigen::Vector3i(len_x, len_y, len_z);

	for (axes[0] = start[0]; axes[0] != end[0]; ++(axes[0]))
	{
		y_start = y_direction * (end[1] - 1) + (1 - y_direction) * start[1];
		y_target = y_direction * (start[1] - 1) + (1 - y_direction) * end[1];
		y_step = -2 * y_direction + 1;

		for (axes[1] = y_start; axes[1] != y_target; axes[1] += y_step)
		{
			z_start = z_direction * (end[2] - 1) + (1 - z_direction) * start[2];
			z_target = z_direction * (start[2] - 1) + (1 - z_direction) * end[2];
			z_step = -2 * z_direction + 1;

			for (axes[2] = z_start; axes[2] != z_target; axes[2] += z_step, ++array_loc)
			{
				grid_loc = (size_t)axes[2] + (size_t)axes[1] * span_y + (size_t)axes[0] * span_x;

				quantized_grid[grid_loc] = presized_block_array[array_loc];
			}

			z_direction = (z_direction == 0);
		}

		y_direction = (y_direction == 0);
	}
}

void VV_TSDF::ExtractBlockUnquantizedS_Traversal(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, double* presized_block_array)
{
	size_t grid_loc;
	size_t array_loc = 0;
	Eigen::Vector3i axes;

	int y_start;
	int y_target;
	int y_step;

	int z_start;
	int z_target;
	int z_step;

	int y_direction = 0;
	int z_direction = 0;

	Eigen::Vector3i start = Eigen::Vector3i(x, y, z);
	Eigen::Vector3i end = start + Eigen::Vector3i(len_x, len_y, len_z);

	for (axes[0] = start[0]; axes[0] != end[0]; ++(axes[0]))
	{
		y_start = y_direction * (end[1] - 1) + (1 - y_direction) * start[1];
		y_target = y_direction * (start[1] - 1) + (1 - y_direction) * end[1];
		y_step = -2 * y_direction + 1;

		for (axes[1] = y_start; axes[1] != y_target; axes[1] += y_step)
		{
			z_start = z_direction * (end[2] - 1) + (1 - z_direction) * start[2];
			z_target = z_direction * (start[2] - 1) + (1 - z_direction) * end[2];
			z_step = -2 * z_direction + 1;

			for (axes[2] = z_start; axes[2] != z_target; axes[2] += z_step, ++array_loc)
			{
				grid_loc = (size_t)axes[2] + (size_t)axes[1] * span_y + (size_t)axes[0] * span_x;

				presized_block_array[array_loc] = double_grid[grid_loc] / gds.unit_length;
			}

			z_direction = (z_direction == 0);
		}

		y_direction = (y_direction == 0);
	}
}

void VV_TSDF::InsertBlockUnquantizedS_Traversal(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, double* presized_block_array)
{
	size_t grid_loc;
	size_t array_loc = 0;
	Eigen::Vector3i axes;

	int y_start;
	int y_target;
	int y_step;

	int z_start;
	int z_target;
	int z_step;

	int y_direction = 0;
	int z_direction = 0;

	Eigen::Vector3i start = Eigen::Vector3i(x, y, z);
	Eigen::Vector3i end = start + Eigen::Vector3i(len_x, len_y, len_z);

	for (axes[0] = start[0]; axes[0] != end[0]; ++(axes[0]))
	{
		y_start = y_direction * (end[1] - 1) + (1 - y_direction) * start[1];
		y_target = y_direction * (start[1] - 1) + (1 - y_direction) * end[1];
		y_step = -2 * y_direction + 1;

		for (axes[1] = y_start; axes[1] != y_target; axes[1] += y_step)
		{
			z_start = z_direction * (end[2] - 1) + (1 - z_direction) * start[2];
			z_target = z_direction * (start[2] - 1) + (1 - z_direction) * end[2];
			z_step = -2 * z_direction + 1;

			for (axes[2] = z_start; axes[2] != z_target; axes[2] += z_step, ++array_loc)
			{
				grid_loc = (size_t)axes[2] + (size_t)axes[1] * span_y + (size_t)axes[0] * span_x;

				double_grid[grid_loc] = presized_block_array[array_loc] * gds.unit_length;
			}

			z_direction = (z_direction == 0);
		}

		y_direction = (y_direction == 0);
	}

	update_quantized = true;
}

void VV_TSDF::SetQuantizedBlock(size_t x, size_t y, size_t z, size_t len_x, size_t len_y, size_t len_z, unsigned char value)
{
#pragma omp parallel for
	for (int loc_x = 0; loc_x < len_x; ++loc_x)
	{
		size_t p_x = (x + (size_t)loc_x) * span_x;

		for (int loc_y = 0; loc_y < len_y; ++loc_y)
		{
			size_t p_y = (y + (size_t)loc_y) * span_y;

			size_t grid_loc = p_x + p_y + z;

			memset(&(quantized_grid[grid_loc]), value, len_z);
		}
	}
}

void VV_TSDF::HarvestSigns(std::vector<unsigned char>& sign_vector)
{
	sign_vector.clear();
	
	size_t new_size_lesser = quantized_grid.size() / 8;
	size_t new_size = new_size_lesser + ((quantized_grid.size() % 8) != 0);
	sign_vector.resize(new_size, 0);

	double average = ((double)min_c + (double)max_c) * 0.5;

	size_t byte = 0;
	size_t byte_mask = 7;

	for (size_t i = 0; i < quantized_grid.size(); ++i)
	{
		byte = i >> 3;

		sign_vector[byte] |= ((unsigned char)(quantized_grid[i] > average) << (i & byte_mask));
	}
}

void VV_TSDF::ApplySigns(std::vector<unsigned char>& sign_vector)
{
	double average = ((double)min_c + (double)max_c) * 0.5;

	unsigned char test_byte = 0;
	size_t byte = 0;
	size_t byte_mask = 7;
	for (size_t i = 0; i < quantized_grid.size(); ++i)
	{
		byte = i >> 3;
		test_byte = sign_vector[byte] & ((unsigned char)1 << (i & byte_mask));

		if ((quantized_grid[i] > average) != (test_byte > 0))
		{
			//unsigned char old_val = quantized_grid[i];
			quantized_grid[i] = max_c - quantized_grid[i];
			//std::cout << "FLIPPED BIT " << GetDecodedGridLoc(i).transpose() << " (" << (int)old_val << " --> " << (int)quantized_grid[i] << ")" << std::endl;
		}

	}

}

void VV_TSDF::PrintCrossSectionOfSolver(size_t y_level)
{
	size_t grid_loc = 0;

	for (size_t x = 0; x < gds.dim_x; ++x)
	{
		for (size_t z = 0; z < gds.dim_z; ++z)
		{
			grid_loc = z + y_level * span_y + x * span_x;

			size_t ind = mc_solver->inds[grid_loc];

			if (ind == SIZE_MAX)
			{
				std::cout << "-";
				continue;
			}

			int z_state = mc_solver->mc_data[ind].vert_indices_on_edges.z();

			if (z_state < 0)
			{
				std::cout << "-";
			}
			else
			{
				std::cout << "O";
			}
		}

		std::cout << std::endl;
	}
}

void VV_TSDF::PrintCrossSectionOfDoubleGrid(size_t y_level)
{
	size_t grid_loc = 0;

	for (size_t x = 0; x < gds.dim_x; ++x)
	{
		for (size_t z = 0; z < gds.dim_z; ++z)
		{
			grid_loc = z + y_level * span_y + x * span_x;

			if (double_grid[grid_loc] > 0)
			{
				std::cout << "-";
			}
			else
			{
				std::cout << "H";
			}
		}

		std::cout << std::endl;
	}
}

void VV_TSDF::PrintCrossSectionOfQuantizedGrid(size_t y_level)
{
	size_t grid_loc = 0;

	for (size_t x = 0; x < gds.dim_x; ++x)
	{
		for (size_t z = 0; z < gds.dim_z; ++z)
		{
			grid_loc = z + y_level * span_y + x * span_x;

			//Reduces 0 to 255 into a single 0 to 9 digit.
			std::cout << ((int)quantized_grid[grid_loc] / 26);
		}

		std::cout << std::endl;
	}
}