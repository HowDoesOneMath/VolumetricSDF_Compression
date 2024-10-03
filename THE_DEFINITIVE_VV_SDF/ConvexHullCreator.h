#pragma once

#include "VV_Mesh.h"

#include <unordered_map>
#include <unordered_set>

class ConvexHullCreator
{
	std::shared_ptr<std::vector<size_t>> GetHullFromPoints(VV_Mesh& mesh,
		std::unordered_set<size_t> &target_points, size_t axis_point_0, size_t axis_point_1);

	std::shared_ptr<std::pair<std::unordered_set<size_t>, std::unordered_set<size_t>>> SplitPointsPerpendicularToAxis(VV_Mesh& mesh,
		std::unordered_set<size_t>& target_points, Eigen::Vector2d axis, size_t separating_index);

	std::shared_ptr<std::pair<std::unordered_set<size_t>, size_t>> GetExteriorPoints(VV_Mesh& mesh,
		std::unordered_set<size_t> &target_points, Eigen::Vector2d* a_0, Eigen::Vector2d* a_1);

public:
	std::shared_ptr<std::vector<size_t>> QuickHullOfUVs(VV_Mesh& mesh);
};