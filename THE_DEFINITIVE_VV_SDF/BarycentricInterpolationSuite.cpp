#include "BarycentricInterpolationSuite.h"

void BarycentricInterpolationSuite::TestInterpolation()
{
	image.assign(image_size, image_size, 1, 3);

	Eigen::Vector2d coord;
	Eigen::Vector3d baryc;

	for (int w = 0; w < image.width(); ++w)
	{
		coord.x() = w;
		coord.x() /= image.width();

		for (int h = 0; h < image.height(); ++h)
		{
			coord.y() = h;
			coord.y() /= image.height();

			if (!PointInsideTriangle(coord, p0, p1, p2))
			{
				continue;
			}

			GetBarycentricCoordinatesOfTriangle<Eigen::Vector2d>(coord, p0, p1, p2, baryc);

			image(w, h, 0, 0) = baryc.x() * 255;
			image(w, h, 0, 1) = baryc.y() * 255;
			image(w, h, 0, 2) = baryc.z() * 255;
		}
	}

	image.save_png(output_image.c_str());
}

void BarycentricInterpolationSuite::TestCGAL()
{
	test_mesh.vertices.elements.push_back(Eigen::Vector3d(p0.x(), p0.y(), 0));
	test_mesh.vertices.elements.push_back(Eigen::Vector3d(p1.x(), p1.y(), 0));
	test_mesh.vertices.elements.push_back(Eigen::Vector3d(p2.x(), p2.y(), 0));

	test_mesh.vertices.indices.push_back(Eigen::Vector3i(0, 1, 2));

	auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(test_mesh.vertices);
	//auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, test_mesh.vertices);
	auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

	//test_mesh.InitializeLocatorCGAL();

	image.assign(image_size, image_size, 1, 3);

	Eigen::Vector2d coord;
	Eigen::Vector3d baryc;

	size_t dummy_index;

	for (int w = 0; w < image.width(); ++w)
	{
		coord.x() = w;
		coord.x() /= image.width();

		for (int h = 0; h < image.height(); ++h)
		{
			coord.y() = h;
			coord.y() /= image.height();

			if (!PointInsideTriangle(coord, p0, p1, p2))
			{
				continue;
			}

			//test_mesh.FindClosestPointCGAL(Eigen::Vector3d(coord.x(), coord.y(), 0), dummy_index, baryc);
			vcm.FindClosestPointCGAL(*cgal_mesh, *aabb_tree, Eigen::Vector3d(coord.x(), coord.y(), 0), dummy_index, baryc);

			image(w, h, 0, 0) = baryc.x() * 255;
			image(w, h, 0, 1) = baryc.y() * 255;
			image(w, h, 0, 2) = baryc.z() * 255;
		}
	}

	//test_mesh.CleanupLocatorCGAL();

	image.save_png(output_image.c_str());
}

void BarycentricInterpolationSuite::run(int argc, char** argv)
{
	//TestInterpolation();

	TestCGAL();
}
