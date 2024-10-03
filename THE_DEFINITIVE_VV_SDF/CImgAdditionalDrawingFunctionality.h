#pragma once

#include <CImg.h>

#include <Eigen/Core>

#include <iostream>

inline void DrawThickLine(cimg_library::CImg<unsigned char>& target_img, Eigen::Vector2d p0, Eigen::Vector2d p1,
	double thickness, Eigen::Vector3<unsigned char> col)
{
	cimg_library::CImg<int> points(4, 2);

	Eigen::Vector2d perp = Eigen::Vector2d(p1.y() - p0.y(), p0.x() - p1.x());
	perp.x() *= target_img.width();
	perp.y() *= target_img.height();
	perp = perp.normalized() * thickness;

	Eigen::Vector2d pix_p0 = Eigen::Vector2d(p0.x() * target_img.width(), p0.y() * target_img.height());
	Eigen::Vector2d pix_p1 = Eigen::Vector2d(p1.x() * target_img.width(), p1.y() * target_img.height());

	Eigen::Vector2d r_00 = pix_p0 + perp;
	Eigen::Vector2d r_01 = pix_p0 - perp;
	Eigen::Vector2d r_10 = pix_p1 + perp;
	Eigen::Vector2d r_11 = pix_p1 - perp;

	points.assign(4, 2);

	points(0, 0) = r_00.x();
	points(0, 1) = r_00.y();
	points(1, 0) = r_01.x();
	points(1, 1) = r_01.y();
	points(3, 0) = r_10.x();
	points(3, 1) = r_10.y();
	points(2, 0) = r_11.x();
	points(2, 1) = r_11.y();

	target_img.draw_polygon(points, col.data(), 1);
}

inline void DrawPolygon(cimg_library::CImg<unsigned char>& target_img, std::vector<Eigen::Vector2d>& points, 
	Eigen::Vector3<unsigned char> col, Eigen::Vector2d scaling, Eigen::Vector2d center, double buffer_size)
{
	cimg_library::CImg<int> p_img(points.size(), 2);

	for (size_t i = 0; i < points.size(); ++i)
	{
		p_img(i, 0) = ((points[i].x() - center.x()) * scaling.x() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.width();
		p_img(i, 1) = ((points[i].y() - center.y()) * scaling.y() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.height();
	}

	target_img.draw_polygon(p_img, col.data(), 1);
}

inline void DrawPolygon(cimg_library::CImg<unsigned char>& target_img, std::vector<Eigen::Vector2d>& points, 
	std::vector<size_t>& indices, Eigen::Vector3<unsigned char> col, Eigen::Vector2d scaling, Eigen::Vector2d center, double buffer_size)
{
	cimg_library::CImg<int> p_img(indices.size(), 2);

	for (size_t i = 0; i < indices.size(); ++i)
	{
		p_img(i, 0) = ((points[indices[i]].x() - center.x()) * scaling.x() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.width();
		p_img(i, 1) = ((points[indices[i]].y() - center.y()) * scaling.y() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.height();
	}

	target_img.draw_polygon(p_img, col.data(), 1);
}

inline void DrawTriangle(cimg_library::CImg<unsigned char>& target_img, Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d p2, 
	Eigen::Vector3<unsigned char> col, Eigen::Vector2d scaling, Eigen::Vector2d center, double buffer_size)
{
	cimg_library::CImg<int> p_img(3, 2);

	p_img(0, 0) = ((p0.x() - center.x()) * scaling.x() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.width();
	p_img(0, 1) = ((p0.y() - center.y()) * scaling.y() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.height();

	p_img(1, 0) = ((p1.x() - center.x()) * scaling.x() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.width();
	p_img(1, 1) = ((p1.y() - center.y()) * scaling.y() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.height();

	p_img(2, 0) = ((p2.x() - center.x()) * scaling.x() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.width();
	p_img(2, 1) = ((p2.y() - center.y()) * scaling.y() * (1.0 - 2.0 * buffer_size) + 0.5) * target_img.height();

	target_img.draw_polygon(p_img, col.data(), 1);
}

inline void DrawTriangle(cimg_library::CImg<unsigned char>& target_img, VV_Mesh &mesh, size_t triangle_index, 
	Eigen::Vector3<unsigned char> col, Eigen::Vector2d scaling, Eigen::Vector2d offset, double buffer_size)
{
	Eigen::Vector3i* tri = &mesh.uvs.indices[triangle_index];
	Eigen::Vector2d* p0 = &mesh.uvs.elements[tri->x()];
	Eigen::Vector2d* p1 = &mesh.uvs.elements[tri->y()];
	Eigen::Vector2d* p2 = &mesh.uvs.elements[tri->z()];
	DrawTriangle(target_img, *p0, *p1, *p2, col, scaling, offset, buffer_size);
}

inline Eigen::Vector2d DrawBoundingBox(cimg_library::CImg<unsigned char>& target_img, VV_Mesh& mesh, size_t axis_p0, size_t axis_p1,
	Eigen::Vector3<unsigned char> col, double thickness, Eigen::Vector2d scaling, Eigen::Vector2d center, double buffer_size)
{
	std::vector<Eigen::Vector2d> scaled_points;
	scaled_points.resize(mesh.uvs.elements.size());

	for (size_t i = 0; i < scaled_points.size(); ++i)
	{
		scaled_points[i] = (mesh.uvs.elements[i] - center).cwiseProduct(scaling * (1.0 - 2.0 * buffer_size)) + Eigen::Vector2d::Ones() * 0.5;
	}

	double leftmost_val = DBL_MAX;
	size_t leftmost_ind = 0;

	double rightmost_val = -DBL_MAX;
	size_t rightmost_ind = 0;

	double upmost_val = -DBL_MAX;
	size_t upmost_ind = 0;

	double edge_dot_p;

	Eigen::Vector2d p0 = scaled_points[axis_p0];
	Eigen::Vector2d p1 = scaled_points[axis_p1];
	Eigen::Vector2d edge = p1 - p0;

	Eigen::Vector2d test_edge;

	for (size_t i = 0; i < scaled_points.size(); ++i)
	{
		test_edge = scaled_points[i] - p0;
		double test_val = edge.dot(test_edge);

		if (test_val < leftmost_val)
		{
			leftmost_val = test_val;
			leftmost_ind = i;
		}

		if (test_val > rightmost_val)
		{
			rightmost_val = test_val;
			rightmost_ind = i;
		}

		test_val = test_edge.squaredNorm() - test_val * test_val / edge.squaredNorm();

		if (test_val > upmost_val)
		{
			upmost_val = test_val;
			upmost_ind = i;
		}
	}

	test_edge = scaled_points[leftmost_ind] - p0;
	Eigen::Vector2d bb_p0 = test_edge.dot(edge) / edge.squaredNorm() * edge + p0;

	test_edge = scaled_points[rightmost_ind] - p0;
	Eigen::Vector2d bb_p1 = test_edge.dot(edge) / edge.squaredNorm() * edge + p0;

	test_edge = scaled_points[upmost_ind] - p0;
	test_edge = test_edge - test_edge.dot(edge) / edge.squaredNorm() * edge;
	Eigen::Vector2d bb_p2 = test_edge + bb_p1;
	Eigen::Vector2d bb_p3 = test_edge + bb_p0;

	//std::cout << "AREA: " << (bb_p1 - bb_p0).norm() * (bb_p2 - bb_p1).norm() << std::endl;

	DrawThickLine(target_img, bb_p0, bb_p1, thickness, col);
	DrawThickLine(target_img, bb_p1, bb_p2, thickness, col);
	DrawThickLine(target_img, bb_p2, bb_p3, thickness, col);
	DrawThickLine(target_img, bb_p3, bb_p0, thickness, col);

	Eigen::Vector2d to_return;

	to_return.x() = (bb_p1 - bb_p0).norm() / (1.0 - 2.0 * buffer_size);
	to_return.y() = (bb_p2 - bb_p1).norm() / (1.0 - 2.0 * buffer_size);

	return to_return;
}