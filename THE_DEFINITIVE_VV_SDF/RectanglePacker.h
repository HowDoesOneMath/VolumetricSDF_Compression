#pragma once

#include <Eigen/Geometry>

#include <set>
#include <vector>
#include <iostream>

template <typename T>
class RectanglePacker
{
public:
	struct RectPairComparator
	{
		bool operator() (const std::pair<size_t, T>& lhs, const std::pair<size_t, T>& rhs) const
		{
			if (lhs.second == rhs.second)
			{
				return lhs.first < rhs.first;
			}

			return lhs.second < rhs.second;
		}
	};

private:
	std::set<std::pair<size_t, T>, RectPairComparator> x_axis_set;
	std::set<std::pair<size_t, T>, RectPairComparator> y_axis_set;
	std::vector<Eigen::Vector2<T>> axis_vector;

	void DebugSingleSet(std::set<std::pair<size_t, T>, RectPairComparator>& to_debug);

	void DebugSets();

	void GetLowestAxes(std::set<std::pair<size_t, T>, RectPairComparator> &axis_set, std::pair<size_t, T> *minimum_array);

	void MergeElements(std::pair<size_t, Eigen::Vector2<T>>* merge, int merge_axis);

public:
	void DebugInstructions(std::vector<std::pair<Eigen::Vector2<T>, Eigen::Vector2i>>& instructions);

	std::shared_ptr<std::vector<std::pair<Eigen::Vector2<T>, Eigen::Vector2i>>> GetMergeOperations(
		std::vector<std::pair<Eigen::Vector2<T>, Eigen::Vector2<T>>> &boxes);
};

template<typename T>
inline void RectanglePacker<T>::DebugSingleSet(std::set<std::pair<size_t, T>, RectPairComparator>& to_debug)
{
	for (auto &kvp : to_debug)
	{
		std::cout << "\t" << kvp.first << ": " << kvp.second << std::endl;
	}
}

template<typename T>
inline void RectanglePacker<T>::DebugSets()
{
	std::cout << "Count: " << x_axis_set.size() << " - " << y_axis_set.size() << std::endl;
	std::cout << "X axis: " << std::endl;
	DebugSingleSet(x_axis_set);
	std::cout << "Y axis: " << std::endl;
	DebugSingleSet(y_axis_set);
}

template<typename T>
inline void RectanglePacker<T>::GetLowestAxes(std::set<std::pair<size_t, T>, RectPairComparator>& axis_set, std::pair<size_t, T>* minimum_array)
{
	size_t axis_counter = 0;

	for (auto kvp : axis_set)
	{
		minimum_array[axis_counter] = kvp;
		++axis_counter;

		if (axis_counter >= 2) break;
	}
}

template<typename T>
inline void RectanglePacker<T>::MergeElements(std::pair<size_t, Eigen::Vector2<T>>* merge, int merge_axis)
{
	int other_axis = 1 - merge_axis;

	//Both axes are treated differently: the major axis is the one along which the two boxes get added together.
	//The first element of the boxes is always the min and the second is always the max.
	//Assuming this is along the x direction, the addition looks as follows:
	// 
	//                        max(b0)   max(b0 + b1)
	//  ^               +--------+      
	//  |               |        |      max(b1)
	//  |               |   b0   |-------+
	//  |               |        |   b1  |
	//  |               |        |       |
	//  |               +--------+-------+ 
	// max(y0, y1)   min(b0)    min(b1)
	//
	//          x0 + x1 ----------------->
	//
	x_axis_set.erase(std::make_pair(merge[0].first, merge[0].second.x()));
	x_axis_set.erase(std::make_pair(merge[1].first, merge[1].second.x()));
	y_axis_set.erase(std::make_pair(merge[0].first, merge[0].second.y()));
	y_axis_set.erase(std::make_pair(merge[1].first, merge[1].second.y()));

	axis_vector[merge[1].first][merge_axis] = merge[1].second[merge_axis] + merge[0].second[merge_axis];
	axis_vector[merge[1].first][other_axis] = std::max(merge[1].second[other_axis], merge[0].second[other_axis]);

	x_axis_set.insert(std::make_pair(merge[1].first, axis_vector[merge[1].first].x()));
	y_axis_set.insert(std::make_pair(merge[1].first, axis_vector[merge[1].first].y()));
}

template<typename T>
inline void RectanglePacker<T>::DebugInstructions(std::vector<std::pair<Eigen::Vector2<T>, Eigen::Vector2i>>& instructions)
{
	std::cout << "\n\nINSTRUCTIONS: " << std::endl;

	for (size_t i = 0; i < instructions.size(); ++i)
	{
		std::cout << "\t" << i << " ---> " << instructions[i].first.transpose() << " ||| " << instructions[i].second.transpose() << std::endl;
	}
}

template<typename T>
inline std::shared_ptr<std::vector<std::pair<Eigen::Vector2<T>, Eigen::Vector2i>>> RectanglePacker<T>::GetMergeOperations(
	std::vector<std::pair<Eigen::Vector2<T>, Eigen::Vector2<T>>>& boxes)
{
	auto to_return = std::make_shared<std::vector<std::pair<Eigen::Vector2<T>, Eigen::Vector2i>>>();
	size_t return_iter = 0;

	size_t box_count = boxes.size();

	to_return->resize(box_count - 1);
	axis_vector.resize(box_count);

	Eigen::Vector2<T> diff;

	for (size_t i = 0; i < box_count; ++i)
	{
		axis_vector[i] = boxes[i].second - boxes[i].first;
		x_axis_set.insert(std::make_pair(i, axis_vector[i].x()));
		y_axis_set.insert(std::make_pair(i, axis_vector[i].y()));
	}

	//DebugSets();

	std::pair<size_t, T> min_x[2];
	std::pair<size_t, T> min_y[2];

	int merge_axis;
	int off_axis;
	std::pair<size_t, Eigen::Vector2<T>> to_merge[2];

	T sum_x;
	T sum_y;

	while (box_count > 1)
	{
		GetLowestAxes(x_axis_set, min_x);
		GetLowestAxes(y_axis_set, min_y);

		sum_x = min_x[0].second + min_x[1].second;
		sum_y = min_y[0].second + min_y[1].second;

		if (sum_x < sum_y)
		{
			to_merge[0].first = min_x[0].first;
			to_merge[1].first = min_x[1].first;

			merge_axis = 0;
		}
		else
		{
			to_merge[0].first = min_y[0].first;
			to_merge[1].first = min_y[1].first;

			merge_axis = 1;
		}

		off_axis = 1 - merge_axis;

		to_merge[0].second = axis_vector[to_merge[0].first];
		to_merge[1].second = axis_vector[to_merge[1].first];

		MergeElements(to_merge, merge_axis);

		(*to_return)[return_iter].second = Eigen::Vector2i(to_merge[0].first, to_merge[1].first);

		(*to_return)[return_iter].first[merge_axis] = to_merge[1].second[merge_axis];
		(*to_return)[return_iter].first[off_axis] = 0;

		--box_count;
		++return_iter;
	}

	x_axis_set.clear();
	y_axis_set.clear();
	axis_vector.clear();

	//DebugInstructions(*to_return);

	return to_return;
}