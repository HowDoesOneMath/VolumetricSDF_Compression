#include "RectanglePackingSuite.h"

void RectanglePackingSuite::GenerateRectanglesRandom()
{
	initial_rectangles.resize(rectangle_count);

	for (size_t i = 0; i < rectangle_count; ++i)
	{
		initial_rectangles[i].first.x() = rw.InRange(rectangle_random_starting_points.x(), rectangle_random_starting_points.y());
		initial_rectangles[i].first.y() = rw.InRange(rectangle_random_starting_points.x(), rectangle_random_starting_points.y());

		initial_rectangles[i].second.x() = rw.InRange(rectangle_random_size_bounds.x(), rectangle_random_size_bounds.y());
		initial_rectangles[i].second.y() = rw.InRange(rectangle_random_size_bounds.x(), rectangle_random_size_bounds.y());

		initial_rectangles[i].second += initial_rectangles[i].first;
	}
}

void RectanglePackingSuite::GenerateRectanglesTriangle()
{
	initial_rectangles.resize(rectangle_count);

	for (size_t i = 0; i < rectangle_count; ++i)
	{
		initial_rectangles[i].first.x() = i;
		initial_rectangles[i].first.y() = i;

		initial_rectangles[i].second.x() = 1;
		initial_rectangles[i].second.y() = i + 1;

		initial_rectangles[i].second += initial_rectangles[i].first;
	}
}

void RectanglePackingSuite::GenerateRectanglesSquare()
{
	initial_rectangles.resize(rectangle_count);

	for (size_t i = 0; i < rectangle_count; ++i)
	{
		initial_rectangles[i].first.x() = i;
		initial_rectangles[i].first.y() = i;

		initial_rectangles[i].second.x() = i + 1;
		initial_rectangles[i].second.y() = i + 1;

		initial_rectangles[i].second += initial_rectangles[i].first;
	}
}

void RectanglePackingSuite::PrintRectangleStats()
{
	std::cout << "Created rects: " << std::endl;

	for (size_t i = 0; i < initial_rectangles.size(); ++i)
	{
		std::cout << "\t" << i << ": " << initial_rectangles[i].first.transpose() << " - " << initial_rectangles[i].second.transpose() <<
			" ---> " << (initial_rectangles[i].second - initial_rectangles[i].first).transpose() << std::endl;
	}
}

void RectanglePackingSuite::PackRectangles()
{
	auto instructions = rp.GetMergeOperations(initial_rectangles);

	rp.DebugInstructions(*instructions);

	size_t instruction_index = instructions->size();

	int box_indices[2];

	Eigen::Vector2i shift;
	Eigen::Vector2i box_dims;

	while (instruction_index > 0)
	{
		--instruction_index;

		box_indices[0] = (*instructions)[instruction_index].second.x();
		box_indices[1] = (*instructions)[instruction_index].second.y();

		shift = (*instructions)[instruction_index].first;

		box_dims = initial_rectangles[box_indices[0]].second - initial_rectangles[box_indices[0]].first;
		initial_rectangles[box_indices[0]].first = initial_rectangles[box_indices[1]].first + shift;
		initial_rectangles[box_indices[0]].second = initial_rectangles[box_indices[0]].first + box_dims;
	}
}

void RectanglePackingSuite::OutputCImgRect()
{
	min_bound.x() = INT32_MAX;
	min_bound.y() = min_bound.x();
	max_bound.x() = INT32_MIN;
	max_bound.y() = max_bound.x();
	for (size_t i = 0; i < initial_rectangles.size(); ++i)
	{
		min_bound = min_bound.cwiseMin(initial_rectangles[i].first);
		max_bound = max_bound.cwiseMax(initial_rectangles[i].second);
	}

	Eigen::Vector2i delta_bound = max_bound - min_bound;

	output_rect.assign(delta_bound.x(), delta_bound.y(), 1, 3);
	output_rect.fill(0);

	double rect_colour;

	size_t total_collision_counts = 0;
	std::vector<size_t> collision_counts;
	collision_counts.resize(initial_rectangles.size(), 0);

	for (size_t i = 0; i < initial_rectangles.size(); ++i)
	{
		initial_rectangles[i].first -= min_bound;
		initial_rectangles[i].second -= min_bound;

		rect_colour = ((double)i / (initial_rectangles.size() - 1)) * 255.0;

		for (size_t w = initial_rectangles[i].first.x(); w < initial_rectangles[i].second.x(); ++w)
		{
			for (size_t h = initial_rectangles[i].first.y(); h < initial_rectangles[i].second.y(); ++h)
			{
				if (output_rect(w, h, 0, 1) != 0)
				{
					++collision_counts[i];
					++total_collision_counts;
				}

				output_rect(w, h, 0, 0) = 255.0 - rect_colour;
				output_rect(w, h, 0, 1) = 127;
				output_rect(w, h, 0, 2) = rect_colour;
			}
		}
	}

	std::cout << "Collision Counts: " << total_collision_counts << std::endl;

	if (total_collision_counts > 0)
	{
		for (size_t i = 0; i < collision_counts.size(); ++i)
		{
			std::cout << "\t" << i << ": " << collision_counts[i] << std::endl;
		}
	}
	else
	{
		output_rect.save_png(rect_save_name.c_str());
	}
}

void RectanglePackingSuite::run(int argc, char** argv)
{
	GenerateRectanglesRandom();
	//GenerateRectanglesTriangle();
	//GenerateRectanglesSquare();

	PrintRectangleStats();

	PackRectangles();

	OutputCImgRect();
}
