#include "SparseMarchingCubesSolver.h"

size_t SparseMarchingCubesSolver::ExtractEdge(std::unordered_map<size_t, VV_SparseTSDF_Stats>& grid, VV_Mesh& to_return, 
	size_t* corners_array, Eigen::Vector3d* positions_array, size_t index_0, size_t index_1, char edge_direction, double threshold)
{
	if (grid[corners_array[index_0]].verts_on_edge[edge_direction] < 0)
	{
		double v0 = abs((double)grid[corners_array[index_0]].value - threshold);
		double v1 = abs((double)grid[corners_array[index_1]].value - threshold);

		double t = v0 / (v0 + v1);

		grid[corners_array[index_0]].verts_on_edge[edge_direction] = to_return.vertices.elements.size();
		to_return.vertices.elements.push_back(positions_array[index_0] * (1.0 - t) + positions_array[index_1] * t);
	}

	return grid[index_0].verts_on_edge[edge_direction];
}

size_t SparseMarchingCubesSolver::ExtractSqrtEdge(std::unordered_map<size_t, VV_SparseTSDF_Stats>& grid, VV_Mesh& to_return, 
	size_t* corners_array, Eigen::Vector3d* positions_array, size_t index_0, size_t index_1, char edge_direction, double threshold)
{
	if (grid[corners_array[index_0]].verts_on_edge[edge_direction] < 0)
	{
		double v0 = abs(sqrt((double)grid[corners_array[index_0]].value) - threshold);
		double v1 = abs(sqrt((double)grid[corners_array[index_1]].value) - threshold);

		double t = v0 / (v0 + v1);

		grid[corners_array[index_0]].verts_on_edge[edge_direction] = to_return.vertices.elements.size();
		to_return.vertices.elements.push_back(positions_array[index_0] * (1.0 - t) + positions_array[index_1] * t);
	}

	return grid[index_0].verts_on_edge[edge_direction];
}

std::shared_ptr<VV_Mesh> SparseMarchingCubesSolver::ExtractMesh(GridDataStruct& gds, std::unordered_map<size_t, VV_SparseTSDF_Stats>& grid,
    double threshold, Eigen::Vector2i uv_divs, double uv_buffer)
{
	Eigen::Vector3i dim_minus_one = Eigen::Vector3i(gds.dim_x - 1, gds.dim_y - 1, gds.dim_z - 1);

	auto to_return = std::make_shared<VV_Mesh>();

	size_t corners[8];
    size_t span_z = 1;
	size_t span_y = gds.dim_z * span_z;
	size_t span_x = gds.dim_y * span_y;

	Eigen::Vector3d positions[8];
	Eigen::Vector3d right = Eigen::Vector3d(gds.unit_length, 0, 0);
	Eigen::Vector3d up = Eigen::Vector3d(0, gds.unit_length, 0);
	Eigen::Vector3d forward = Eigen::Vector3d(0, 0, gds.unit_length);

	size_t edges[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };

	size_t visited_cells = 0;

	for (size_t x = 0; x < dim_minus_one[0]; ++x)
	{
		for (size_t y = 0; y < dim_minus_one[1]; ++y)
		{
			for (size_t z = 0; z < dim_minus_one[2]; ++z)
			{
                corners[0] = z + y * span_y + x * span_x;
                corners[4] = corners[0] + span_z;
                corners[2] = corners[0] + span_y;
                corners[6] = corners[2] + span_z;
                corners[1] = corners[0] + span_x;
                corners[5] = corners[1] + span_z;
                corners[3] = corners[1] + span_y;
                corners[7] = corners[3] + span_z;

                int index = 0;

                index |= 1 *    (grid[corners[0]].value < threshold);
                index |= 2 *    (grid[corners[1]].value < threshold);
                index |= 8 *    (grid[corners[2]].value < threshold);
                index |= 4 *    (grid[corners[3]].value < threshold);
                index |= 16 *   (grid[corners[4]].value < threshold);
                index |= 32 *   (grid[corners[5]].value < threshold);
                index |= 128 *  (grid[corners[6]].value < threshold);
                index |= 64 *   (grid[corners[7]].value < threshold);

                index = 255 - index;

                if (!MarchingTables::edge_table[index])
                    continue;

                //for (int i = 0; i < 8; ++i)
                //{
                //    if (inds[corners[i]] == SIZE_MAX)
                //    {
                //        CreateNewIndex(corners[i]);
                //    }
                //}
                //
                //
                //++visited_cells;
                //
                //positions[0] = Eigen::Vector3d(x, y, z) * gds.unit_length + grid_lower_bound;
                //positions[4] = positions[0] + forward;
                //positions[2] = positions[0] + up;
                //positions[6] = positions[2] + forward;
                //positions[1] = positions[0] + right;
                //positions[5] = positions[1] + forward;
                //positions[3] = positions[1] + up;
                //positions[7] = positions[3] + forward;
                //
                //if ((MarchingTables::edge_table[index] & 1) > 0)
                //    edges[0] = ExtractEdge(data, *to_return, corners, positions, 0, 1, 0, threshold);
                //if ((MarchingTables::edge_table[index] & 2) > 0)
                //    edges[1] = ExtractEdge(data, *to_return, corners, positions, 1, 3, 1, threshold);
                //if ((MarchingTables::edge_table[index] & 4) > 0)
                //    edges[2] = ExtractEdge(data, *to_return, corners, positions, 2, 3, 0, threshold);
                //if ((MarchingTables::edge_table[index] & 8) > 0)
                //    edges[3] = ExtractEdge(data, *to_return, corners, positions, 0, 2, 1, threshold);
                //if ((MarchingTables::edge_table[index] & 16) > 0)
                //    edges[4] = ExtractEdge(data, *to_return, corners, positions, 4, 5, 0, threshold);
                //if ((MarchingTables::edge_table[index] & 32) > 0)
                //    edges[5] = ExtractEdge(data, *to_return, corners, positions, 5, 7, 1, threshold);
                //if ((MarchingTables::edge_table[index] & 64) > 0)
                //    edges[6] = ExtractEdge(data, *to_return, corners, positions, 6, 7, 0, threshold);
                //if ((MarchingTables::edge_table[index] & 128) > 0)
                //    edges[7] = ExtractEdge(data, *to_return, corners, positions, 4, 6, 1, threshold);
                //if ((MarchingTables::edge_table[index] & 256) > 0)
                //    edges[8] = ExtractEdge(data, *to_return, corners, positions, 0, 4, 2, threshold);
                //if ((MarchingTables::edge_table[index] & 512) > 0)
                //    edges[9] = ExtractEdge(data, *to_return, corners, positions, 1, 5, 2, threshold);
                //if ((MarchingTables::edge_table[index] & 1024) > 0)
                //    edges[10] = ExtractEdge(data, *to_return, corners, positions, 3, 7, 2, threshold);
                //if ((MarchingTables::edge_table[index] & 2048) > 0)
                //    edges[11] = ExtractEdge(data, *to_return, corners, positions, 2, 6, 2, threshold);
                //
                //auto tri_table_seg = MarchingTables::tri_table[index];
                //
                //mc_data[inds[corners[0]]].triangle_indices_start = to_return->vertices.indices.size();
                //
                ////Adding triangles to the mesh
                //for (int i = 0; tri_table_seg[i] != -1; i += 3)
                //{
                //    to_return->vertices.indices.push_back(Eigen::Vector3i(edges[tri_table_seg[i]], edges[tri_table_seg[i + 1]], edges[tri_table_seg[i + 2]]));
                //    ++mc_data[inds[corners[0]]].triangle_count;
                //}
			}
		}
	}

	return to_return;
}
