#pragma once

#include "VV_Mesh.h"
#include "GridDataStruct.h"
#include "VV_SparseTSDF_Stats.h"
#include "MarchingTables.h"

#include <unordered_map>

class SparseMarchingCubesSolver
{
	size_t ExtractEdge(std::unordered_map<size_t, VV_SparseTSDF_Stats>& grid, VV_Mesh& to_return, size_t* corners_array, Eigen::Vector3d* positions_array,
		size_t index_0, size_t index_1, char edge_direction, double threshold);

	size_t ExtractSqrtEdge(std::unordered_map<size_t, VV_SparseTSDF_Stats>& grid, VV_Mesh& to_return, size_t* corners_array, Eigen::Vector3d* positions_array,
		size_t index_0, size_t index_1, char edge_direction, double threshold);

	std::shared_ptr<VV_Mesh> ExtractMesh(GridDataStruct& gds, std::unordered_map<size_t, VV_SparseTSDF_Stats> &grid, 
		double threshold, Eigen::Vector2i uv_divs, double uv_buffer);
};