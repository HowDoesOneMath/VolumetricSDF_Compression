#include "AbstractAttribute.h"

//const int AbstractAttribute::division_table[8][13] = {
//		{0, 1, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
//		{0, 3, 2, 3, 1, 2, -1, -1, -1, -1, -1, -1, -1},
//		{0, 1, 4, 0, 4, 2, -1, -1, -1, -1, -1, -1, -1},
//		{0, 3, 2, 3, 4, 2, 3, 1, 4, -1, -1, -1, -1},
//		{0, 1, 5, 1, 2, 5, -1, -1, -1, -1, -1, -1, -1},
//		{0, 3, 5, 3, 1, 5, 1, 2, 5, -1, -1, -1, -1},
//		{0, 1, 4, 0, 4, 5, 4, 2, 5, -1, -1, -1, -1},
//		{0, 3, 5, 3, 1, 4, 4, 2, 5, 3, 4, 5, -1}
//};

size_t AbstractAttribute::GetOrGenerateMidpointIndex(std::vector<std::pair<size_t, size_t>>& midpoints, size_t current_attribute, size_t to_find)
{
	for (int i = 0; i < midpoints.size(); ++i)
	{
		if (midpoints[i].first == to_find)
		{
			return midpoints[i].second;
		}
	}

	midpoints.push_back(std::make_pair(to_find, CreateNewMidpointElement(current_attribute, to_find)));

	return midpoints.back().second;
}

void AbstractAttribute::SubdivideTriangle(size_t index, size_t old_indices_size, std::vector<std::vector<std::pair<size_t, size_t>>>& midpoints)
{
	int first_index_of_new_triangles = 3 * index + old_indices_size;

	bool comparison_lesser_01 = (indices[index].x() < indices[index].y());
	bool comparison_lesser_12 = (indices[index].y() < indices[index].z());
	bool comparison_lesser_20 = (indices[index].z() < indices[index].x());

	Eigen::Vector3i lesser_xy_yz_zx = Eigen::Vector3i(
		(comparison_lesser_01 ? indices[index].x() : indices[index].y()),
		(comparison_lesser_12 ? indices[index].y() : indices[index].z()),
		(comparison_lesser_20 ? indices[index].z() : indices[index].x())
		);

	Eigen::Vector3i greater_xy_yz_zx = Eigen::Vector3i(
		(comparison_lesser_01 ? indices[index].y() : indices[index].x()),
		(comparison_lesser_12 ? indices[index].z() : indices[index].y()),
		(comparison_lesser_20 ? indices[index].x() : indices[index].z())
		);

	Eigen::Vector3i midpoint_index_xy_yz_zx = Eigen::Vector3i(
		GetOrGenerateMidpointIndex(midpoints[lesser_xy_yz_zx.x()], lesser_xy_yz_zx.x(), greater_xy_yz_zx.x()),
		GetOrGenerateMidpointIndex(midpoints[lesser_xy_yz_zx.y()], lesser_xy_yz_zx.y(), greater_xy_yz_zx.y()),
		GetOrGenerateMidpointIndex(midpoints[lesser_xy_yz_zx.z()], lesser_xy_yz_zx.z(), greater_xy_yz_zx.z())
	);

	indices[first_index_of_new_triangles] = Eigen::Vector3i(indices[index].x(), midpoint_index_xy_yz_zx.x(), midpoint_index_xy_yz_zx.z());

	indices[first_index_of_new_triangles + 1] = Eigen::Vector3i(indices[index].y(), midpoint_index_xy_yz_zx.y(), midpoint_index_xy_yz_zx.x());

	indices[first_index_of_new_triangles + 2] = Eigen::Vector3i(indices[index].z(), midpoint_index_xy_yz_zx.z(), midpoint_index_xy_yz_zx.y());

	indices[index] = midpoint_index_xy_yz_zx;
}

size_t AbstractAttribute::ClearNegativeTriangles()
{
	size_t negative_count = 0;

	bool bad_triangle = false;

	for (size_t i = 0; i < indices.size(); )
	{
		bad_triangle = ((indices[i].x() < 0) && (indices[i].y() < 0) && (indices[i].z() < 0));

		if (bad_triangle)
		{
			indices[i] = indices.back();
			indices.pop_back();
			++negative_count;
		}
		else
		{
			++i;
		}
	}

	return negative_count;
}

std::shared_ptr<std::vector<std::vector<std::pair<size_t, size_t>>>> AbstractAttribute::SubdivideAttributeAndReturnMidpoints()
{
	auto to_return = std::make_shared<std::vector<std::vector<std::pair<size_t, size_t>>>>();// midpoints;
	to_return->resize(GetElementCount());

	for (int i = 0; i < to_return->size(); ++i)
	{
		(*to_return)[i].push_back(std::make_pair(i, i));
	}

	size_t initial_index_count = indices.size();
	indices.resize(initial_index_count * 4);

	for (size_t i = 0; i < initial_index_count; ++i)
	{
		SubdivideTriangle(i, initial_index_count, *to_return);
	}

	return to_return;
}

void AbstractAttribute::Clear()
{
	indices.clear();
	ClearElements();
}
