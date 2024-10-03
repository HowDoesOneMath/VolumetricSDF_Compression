#include "ConvexHullSuite.h"

std::shared_ptr<std::vector<Eigen::Vector2d>> ConvexHullSuite::ConstructRoughBoundingBox(VV_Mesh& input_mesh, std::vector<size_t>& hull_points, size_t target_edge)
{
	auto to_return = std::make_shared<std::vector<Eigen::Vector2d>>();
	to_return->resize(4);

	size_t next_vert = (target_edge + 1) % hull_points.size();

	Eigen::Vector2d edge = input_mesh.uvs.elements[hull_points[next_vert]] - input_mesh.uvs.elements[hull_points[target_edge]];

	size_t furthest_left = 0;
	double leftmost_value = DBL_MAX;

	size_t furthest_right = 0;
	double rightmost_value = -DBL_MAX;

	size_t furthest_up = 0;
	double upmost_value_sqr = -DBL_MIN;

	Eigen::Vector2d test_edge;
	double test_val;

	for (size_t i = 0; i < input_mesh.uvs.elements.size(); ++i)
	{
		test_edge = test_mesh.uvs.elements[i] - input_mesh.uvs.elements[hull_points[target_edge]];
		test_val = edge.dot(test_edge);

		if (test_val < leftmost_value)
		{
			leftmost_value = test_val;
			furthest_left = i;
		}

		if (test_val > rightmost_value)
		{
			rightmost_value = test_val;
			furthest_right = i;
		}

		test_val = test_edge.squaredNorm() - test_val * test_val / edge.squaredNorm();

		if (test_val > upmost_value_sqr)
		{
			upmost_value_sqr = test_val;
			furthest_up = i;
		}
	}

	test_edge = test_mesh.uvs.elements[furthest_left] - input_mesh.uvs.elements[hull_points[target_edge]];
	(*to_return)[0] = test_edge.dot(edge) / edge.squaredNorm() * edge + input_mesh.uvs.elements[hull_points[target_edge]];

	test_edge = test_mesh.uvs.elements[furthest_right] - input_mesh.uvs.elements[hull_points[target_edge]];
	(*to_return)[1] = test_edge.dot(edge) / edge.squaredNorm() * edge + input_mesh.uvs.elements[hull_points[target_edge]];

	test_edge = test_mesh.uvs.elements[furthest_up] - input_mesh.uvs.elements[hull_points[target_edge]];
	test_edge = test_edge - test_edge.dot(edge) / edge.squaredNorm() * edge;
	(*to_return)[2] = test_edge + (*to_return)[1];
	(*to_return)[3] = test_edge + (*to_return)[0];

	return to_return;
}

void ConvexHullSuite::QuickGenerateIndicesUVs(VV_Mesh& mesh)
{
	mesh.uvs.indices.clear();

	for (size_t i = 2; i < mesh.uvs.elements.size(); ++i)
	{
		mesh.uvs.indices.push_back(Eigen::Vector3i(0, i - 1, i));
	}
}

void ConvexHullSuite::CreateTestPoints_TwoSinWaves(size_t density)
{
	test_mesh.uvs.indices.clear();
	test_mesh.uvs.elements.clear();

	double i_pos;
	double i_pi;

	double sin_period = 3.75;

	double sin_offset = 0.8;
	double sin_mult = (1.0 - sin_offset);
	
	double buffer_offset = 0.176;
	double buffer_mult = (1.0 - 2 * buffer_offset);

	Eigen::Vector2d point;

	for (size_t i = 0; i < density; ++i)
	{
		i_pos = ((double)i / density);
		i_pi = i_pos * J_PI;

		point.x() = i_pos * buffer_mult + buffer_offset;
		point.y() = (sin_mult * sin(i_pi * sin_period) + sin_offset) * buffer_mult + buffer_offset;

		test_mesh.uvs.elements.push_back(point);
	}

	for (size_t i = 0; i < density; ++i)
	{
		i_pos = ((double)i / density);
		i_pi = i_pos * J_PI;

		point.x() = (1.0 - i_pos) * buffer_mult + buffer_offset;
		point.y() = (-sin_mult * sin(i_pi * sin_period) + sin_mult) * buffer_mult + buffer_offset;

		test_mesh.uvs.elements.push_back(point);
	}

	QuickGenerateIndicesUVs(test_mesh);
}

void ConvexHullSuite::CreateTestPoints_SinFlower(size_t density)
{
	test_mesh.uvs.indices.clear();
	test_mesh.uvs.elements.clear();

	int petal_count = 7;

	double sin_middle = 0.25;
	double sin_scale = 0.2;

	double angle;
	double sin_strength;

	Eigen::Vector2d center = Eigen::Vector2d(0.5, 0.5);
	Eigen::Vector2d point;

	double angle_offset = J_PI * 0.25;
	double ovaloid_squish;

	for (size_t i = 0; i < density; ++i)
	{
		angle = (i * J_PI * 2) / density;
		sin_strength = sin_scale * sin(angle * petal_count) + sin_middle;

		ovaloid_squish = sin(angle_offset + angle);
		ovaloid_squish = (ovaloid_squish * ovaloid_squish) * 0.5 + 0.5;

		point.x() = sin_strength * sin(angle) * ovaloid_squish + center.x();
		point.y() = sin_strength * cos(angle) * ovaloid_squish + center.y();

		test_mesh.uvs.elements.push_back(point);
	}

	QuickGenerateIndicesUVs(test_mesh);
}

void ConvexHullSuite::TestConvexHull()
{
	auto hull_points = chc.QuickHullOfUVs(test_mesh);

	auto edges = rc.GetHullEdges(test_mesh, *hull_points);
	auto antipodal_points = rc.GetAntipodalPoints(*edges);

	for (size_t i = 0; i < antipodal_points->size(); ++i)
	{
		std::cout << i << ": " << (*antipodal_points)[i].second.first << ", " << (*antipodal_points)[i].second.second << std::endl;
	}

	VV_Mesh convex_hull;
	convex_hull.uvs.elements.resize(hull_points->size());

	for (size_t i = 0; i < hull_points->size(); ++i)
	{
		convex_hull.uvs.elements[i] = test_mesh.uvs.elements[(*hull_points)[i]];
	}

	QuickGenerateIndicesUVs(convex_hull);

	//for (size_t i = 0; i < chc.quickhull_tris.size(); ++i)
	//{
	//	std::cout << i << "; " << chc.quickhull_tris[i].transpose() << std::endl;
	//}

	std::cout << "'Mesh' points: " << test_mesh.uvs.elements.size() << std::endl;
	std::cout << "Convex Hull points: " << convex_hull.uvs.elements.size() << std::endl;

	cimg_library::CImg<unsigned char> new_img;
	cimg_library::CImg<unsigned char> shade_img;

	new_img.assign(img_size, img_size, 1, 3);
	new_img.fill(0);

	shade_img.assign(new_img.width(), new_img.height(), new_img.depth(), new_img.spectrum());
	shade_img.fill(0);

	Eigen::Vector3<unsigned char> shade_col;

	shade_col = Eigen::Vector3<unsigned char>(0, 255, 255);
	DrawPolygon(new_img, convex_hull.uvs.elements, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);

	shade_col = Eigen::Vector3<unsigned char>(255, 0, 0);
	DrawPolygon(new_img, test_mesh.uvs.elements, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);

	shade_col.x() = 0;
	shade_col.y() = 127;
	shade_col.z() = 255;
	DrawPolygon(shade_img, test_mesh.uvs.elements, *hull_points, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);

	Eigen::Vector2d m_p0;
	Eigen::Vector2d m_p1;

	for (size_t i = 0; i < antipodal_points->size(); ++i)
	{
		auto point_pair = (*antipodal_points)[i];

		shade_col.x() = (255 * i) / (antipodal_points->size() - 1);
		shade_col.y() = 127;
		shade_col.z() = 127;

		m_p0 = convex_hull.uvs.elements[point_pair.second.first];
		m_p1 = convex_hull.uvs.elements[point_pair.second.second];

		DrawThickLine(shade_img, m_p0, m_p1, line_thickness, shade_col);

		m_p0 = convex_hull.uvs.elements[(point_pair.first + 1) % convex_hull.uvs.elements.size()];
		m_p1 = convex_hull.uvs.elements[point_pair.first];

		DrawThickLine(shade_img, m_p0, m_p1, line_thickness, shade_col);
	}

	new_img.append(shade_img, 'x');

	new_img.save_png(test_output_image.c_str());
}

void ConvexHullSuite::TestAutomaticOrientation()
{
	auto hull_points = chc.QuickHullOfUVs(test_mesh);

	auto edges = rc.GetHullEdges(test_mesh, *hull_points);
	auto antipodal_points = rc.GetAntipodalPoints(*edges);

	for (size_t i = 0; i < antipodal_points->size(); ++i)
	{
		std::cout << i << ": " << (*antipodal_points)[i].second.first << ", " << (*antipodal_points)[i].second.second << std::endl;
	}

	VV_Mesh convex_hull;
	convex_hull.uvs.elements.resize(hull_points->size());

	for (size_t i = 0; i < hull_points->size(); ++i)
	{
		convex_hull.uvs.elements[i] = test_mesh.uvs.elements[(*hull_points)[i]];
	}

	QuickGenerateIndicesUVs(convex_hull);

	auto rotated_mesh = test_mesh.GetCopy();
	auto bounding_box = rc.RotateUVs(*rotated_mesh);

	std::cout << "'Mesh' points: " << test_mesh.uvs.elements.size() << std::endl;
	std::cout << "Convex Hull points: " << convex_hull.uvs.elements.size() << std::endl;

	cimg_library::CImg<unsigned char> new_img;
	cimg_library::CImg<unsigned char> rotated_img;

	new_img.assign(img_size, img_size, 1, 3);
	new_img.fill(0);

	rotated_img.assign(new_img.width(), new_img.height(), new_img.depth(), new_img.spectrum());
	rotated_img.fill(0);

	Eigen::Vector3<unsigned char> shade_col;
	cimg_library::CImg<int> points;


	shade_col = Eigen::Vector3<unsigned char>(0, 127, 255);
	DrawPolygon(new_img, test_mesh.uvs.elements, *hull_points, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);


	shade_col = Eigen::Vector3<unsigned char>(255, 0, 0);
	DrawPolygon(new_img, test_mesh.uvs.elements, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);

	Eigen::Vector2d min_offset = Eigen::Vector2d(DBL_MAX, DBL_MAX);
	Eigen::Vector2d max_offset = Eigen::Vector2d(-DBL_MAX, -DBL_MAX);

	for (size_t i = 0; i < rotated_mesh->uvs.elements.size(); ++i)
	{
		min_offset = min_offset.cwiseMin(rotated_mesh->uvs.elements[i]);
		max_offset = max_offset.cwiseMax(rotated_mesh->uvs.elements[i]);
	}

	Eigen::Vector2d centerpoint = (max_offset + min_offset) * 0.5;
	Eigen::Vector2d to_image_center = centerpoint - Eigen::Vector2d::Ones() * 0.5;

	for (size_t i = 0; i < rotated_mesh->uvs.elements.size(); ++i)
	{
		rotated_mesh->uvs.elements[i] -= to_image_center;
	}

	bounding_box.first -= to_image_center;
	bounding_box.second -= to_image_center;

	min_offset -= to_image_center;
	max_offset -= to_image_center;


	double bb_area;
	double bb_width;
	double bb_height;

	double least_bb_area = DBL_MAX;
	size_t least_bb = 0;

	for (size_t i = 0; i < hull_points->size(); ++i)
	{
		auto bb_vector = ConstructRoughBoundingBox(test_mesh, *hull_points, i);

		bb_width = ((*bb_vector)[0] - (*bb_vector)[1]).squaredNorm();
		bb_height = ((*bb_vector)[1] - (*bb_vector)[2]).squaredNorm();
		bb_area = bb_width * bb_height;

		std::cout << "BB square area of " << i << ": " << bb_area << " ----> H: " << bb_width << ", W: " << bb_height << std::endl;

		if (bb_area < least_bb_area)
		{
			least_bb_area = bb_area;
			least_bb = i;
		}
	}

	std::cout << "Least BB square area: " << least_bb << " --> " << least_bb_area << std::endl;

	for (size_t i = 0; i < hull_points->size(); ++i)
	{
		shade_col = Eigen::Vector3<unsigned char>(0, (127.0 * i) / (hull_points->size() - 1), 127);

		auto bb_vector = ConstructRoughBoundingBox(test_mesh, *hull_points, i);

		for (size_t j = 0; j < bb_vector->size(); ++j)
		{
			size_t next_p = (j + 1) % bb_vector->size();

			DrawThickLine(new_img, (*bb_vector)[j], (*bb_vector)[next_p], line_thickness, shade_col);
		}
	}

	shade_col = Eigen::Vector3<unsigned char>(255, 127, 0);
	auto least_bb_vector = ConstructRoughBoundingBox(test_mesh, *hull_points, least_bb);
	for (size_t j = 0; j < least_bb_vector->size(); ++j)
	{
		size_t next_p = (j + 1) % least_bb_vector->size();

		DrawThickLine(new_img, (*least_bb_vector)[j], (*least_bb_vector)[next_p], line_thickness, shade_col);
	}


	shade_col = Eigen::Vector3<unsigned char>(255, 127, 0);
	DrawPolygon(rotated_img, rotated_mesh->uvs.elements, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);


	Eigen::Vector2d bb_point_0;
	Eigen::Vector2d bb_point_1;
	
	bb_point_1 = bounding_box.first;
	shade_col = Eigen::Vector3<unsigned char>(255, 255, 255);

	bb_point_0 = Eigen::Vector2d(bounding_box.first.x(), bounding_box.second.y());
	DrawThickLine(rotated_img, bb_point_0, bb_point_1, line_thickness, shade_col);
	bb_point_1 = bb_point_0;

	bb_point_0 = bounding_box.second;
	DrawThickLine(rotated_img, bb_point_0, bb_point_1, line_thickness, shade_col);
	bb_point_1 = bb_point_0;

	bb_point_0 = Eigen::Vector2d(bounding_box.second.x(), bounding_box.first.y());
	DrawThickLine(rotated_img, bb_point_0, bb_point_1, line_thickness, shade_col);
	bb_point_1 = bb_point_0;

	bb_point_0 = bounding_box.first;
	DrawThickLine(rotated_img, bb_point_0, bb_point_1, line_thickness, shade_col);
	bb_point_1 = bb_point_0;


	bb_point_1 = min_offset;
	shade_col = Eigen::Vector3<unsigned char>(127, 127, 0);

	bb_point_0 = Eigen::Vector2d(min_offset.x(), max_offset.y());
	DrawThickLine(rotated_img, bb_point_0, bb_point_1, line_thickness - 1, shade_col);
	bb_point_1 = bb_point_0;

	bb_point_0 = max_offset;
	DrawThickLine(rotated_img, bb_point_0, bb_point_1, line_thickness - 1, shade_col);
	bb_point_1 = bb_point_0;

	bb_point_0 = Eigen::Vector2d(max_offset.x(), min_offset.y());
	DrawThickLine(rotated_img, bb_point_0, bb_point_1, line_thickness - 1, shade_col);
	bb_point_1 = bb_point_0;

	bb_point_0 = min_offset;
	DrawThickLine(rotated_img, bb_point_0, bb_point_1, line_thickness - 1, shade_col);
	bb_point_1 = bb_point_0;


	new_img.append(rotated_img, 'x');

	new_img.save_png(test_output_image.c_str());
}

void ConvexHullSuite::TestFindPerpendiculars()
{
	auto hull_points = chc.QuickHullOfUVs(test_mesh);

	auto edges = rc.GetHullEdges(test_mesh, *hull_points);
	auto antipodal_points = rc.GetAntipodalPoints(*edges);

	for (size_t i = 0; i < antipodal_points->size(); ++i)
	{
		std::cout << i << ": " << (*antipodal_points)[i].second.first << ", " << (*antipodal_points)[i].second.second << std::endl;
	}

	VV_Mesh convex_hull;
	convex_hull.uvs.elements.resize(hull_points->size());

	for (size_t i = 0; i < hull_points->size(); ++i)
	{
		convex_hull.uvs.elements[i] = test_mesh.uvs.elements[(*hull_points)[i]];
	}

	QuickGenerateIndicesUVs(convex_hull);

	auto rotated_mesh = test_mesh.GetCopy();
	auto bounding_box = rc.RotateUVs(*rotated_mesh);

	std::cout << "'Mesh' points: " << test_mesh.uvs.elements.size() << std::endl;
	std::cout << "Convex Hull points: " << convex_hull.uvs.elements.size() << std::endl;

	cimg_library::CImg<unsigned char> new_img;
	cimg_library::CImg<unsigned char> rotated_img;
	cimg_library::CImg<unsigned char> shade_img;

	new_img.assign(img_size, img_size, 1, 3);
	new_img.fill(0);

	rotated_img.assign(new_img.width(), new_img.height(), new_img.depth(), new_img.spectrum());
	rotated_img.fill(0);

	Eigen::Vector3<unsigned char> shade_col;
	cimg_library::CImg<int> points;


	shade_col = Eigen::Vector3<unsigned char>(0, 127, 255);
	DrawPolygon(new_img, test_mesh.uvs.elements, *hull_points, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);


	shade_col = Eigen::Vector3<unsigned char>(255, 0, 0);
	DrawPolygon(new_img, test_mesh.uvs.elements, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);


	Eigen::Vector2d min_offset = Eigen::Vector2d(DBL_MAX, DBL_MAX);
	Eigen::Vector2d max_offset = Eigen::Vector2d(-DBL_MAX, -DBL_MAX);

	for (size_t i = 0; i < rotated_mesh->uvs.elements.size(); ++i)
	{
		min_offset = min_offset.cwiseMin(rotated_mesh->uvs.elements[i]);
		max_offset = max_offset.cwiseMax(rotated_mesh->uvs.elements[i]);
	}

	Eigen::Vector2d centerpoint = (max_offset + min_offset) * 0.5;
	Eigen::Vector2d to_image_center = centerpoint - Eigen::Vector2d::Ones() * 0.5;

	for (size_t i = 0; i < rotated_mesh->uvs.elements.size(); ++i)
	{
		rotated_mesh->uvs.elements[i] -= to_image_center;
	}

	bounding_box.first -= to_image_center;
	bounding_box.second -= to_image_center;

	min_offset -= to_image_center;
	max_offset -= to_image_center;


	shade_col = Eigen::Vector3<unsigned char>(255, 127, 0);
	DrawPolygon(rotated_img, rotated_mesh->uvs.elements, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);

	for (size_t i = 0; i < antipodal_points->size(); ++i)
	{
		auto point_pair = (*antipodal_points)[i];

		shade_col.x() = (255 * i) / (antipodal_points->size() - 1);
		shade_col.y() = 127;
		shade_col.z() = 127;

		Eigen::Vector2d m_p0 = convex_hull.uvs.elements[point_pair.second.first];
		Eigen::Vector2d m_p1 = convex_hull.uvs.elements[point_pair.second.second];

		DrawThickLine(new_img, m_p0, m_p1, line_thickness * 0.5, shade_col);

		m_p0 = convex_hull.uvs.elements[(point_pair.first + 1) % convex_hull.uvs.elements.size()];
		m_p1 = convex_hull.uvs.elements[point_pair.first];

		DrawThickLine(new_img, m_p0, m_p1, line_thickness * 0.5, shade_col);
	}


	std::vector<Eigen::Vector2d> antipodal_lengths;
	antipodal_lengths.resize(antipodal_points->size());
	for (size_t i = 0; i < antipodal_lengths.size(); ++i)
	{
		antipodal_lengths[i] = test_mesh.uvs.elements[(*hull_points)[(*antipodal_points)[i].second.second]]
			- test_mesh.uvs.elements[(*hull_points)[(*antipodal_points)[i].second.first]];
	}

	double greatest_length = 0;
	size_t chosen_length = 0;

	for (size_t i = 0; i < edges->size(); ++i)
	{
		shade_img.assign(new_img.width(), new_img.height(), new_img.depth(), new_img.spectrum());
		shade_img.fill(0);

		shade_col.x() = 255;
		shade_col.y() = 255;
		shade_col.z() = 255;
		DrawPolygon(shade_img, test_mesh.uvs.elements, *hull_points, shade_col, Eigen::Vector2d::Ones(), Eigen::Vector2d::Ones() * 0.5, 0);

		size_t next_p = (i + 1) % edges->size();

		greatest_length = -DBL_MAX;
		for (size_t j = 0; j < antipodal_lengths.size(); ++j)
		{
			double new_length = (*edges)[i].dot(antipodal_lengths[j]);
			new_length = (new_length * new_length) / (*edges)[i].squaredNorm();

			if (new_length > greatest_length)
			{
				chosen_length = j;
				greatest_length = new_length;
			}
		}

		shade_col.x() = 255;
		shade_col.y() = 0;
		shade_col.z() = 0;

		DrawThickLine(shade_img, test_mesh.uvs.elements[(*hull_points)[i]], test_mesh.uvs.elements[(*hull_points)[next_p]], line_thickness, shade_col);
		DrawThickLine(shade_img, test_mesh.uvs.elements[(*hull_points)[(*antipodal_points)[chosen_length].second.first]],
			test_mesh.uvs.elements[(*hull_points)[(*antipodal_points)[chosen_length].second.second]], line_thickness, shade_col);

		rotated_img.append(shade_img, 'x');
	}

	new_img.append(rotated_img, 'x');

	new_img.save_png(test_output_image.c_str());
}

void ConvexHullSuite::run(int argc, char** argv)
{
	//CreateTestPoints_TwoSinWaves(test_density);
	CreateTestPoints_SinFlower(test_density);

	//TestConvexHull();
	TestAutomaticOrientation();
	//TestFindPerpendiculars();
}
