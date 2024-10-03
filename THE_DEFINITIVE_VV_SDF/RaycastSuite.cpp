#include "RaycastSuite.h"

void RaycastSuite::InitializeTransform()
{
	std::cout << transform.matrix() << std::endl << std::endl;

	transform
		.rotate(Eigen::AngleAxisd(euler_rotation.y() * to_radians, Eigen::Vector3d::UnitY()))
		.rotate(Eigen::AngleAxisd(euler_rotation.x() * to_radians, Eigen::Vector3d::UnitX()))
		.rotate(Eigen::AngleAxisd(euler_rotation.z() * to_radians, Eigen::Vector3d::UnitZ()));

	std::cout << transform.matrix() << std::endl << std::endl;

	std::cout << "Corner 0: " << (transform.inverse() * p0).transpose() << std::endl;
	std::cout << "Corner 1: " << (transform.inverse() * p1).transpose() << std::endl;
	std::cout << "Corner 2: " << (transform.inverse() * p2).transpose() << std::endl;

	std::cout << "Ray direction: " << Eigen::Vector3d::UnitZ().transpose() << std::endl;
	std::cout << "Ray direction transformed: " << (transform * Eigen::Vector3d::UnitZ()).transpose() << std::endl;
}

void RaycastSuite::TestMultiCast()
{
	raycast_canvas.assign(pixels_w, pixels_h, 1, 3);
	raycast_canvas.fill(0);

	Eigen::Vector3d r = transform * Eigen::Vector3d::UnitX();
	Eigen::Vector3d u = transform * Eigen::Vector3d::UnitY();

	Eigen::Vector3d dir = transform * Eigen::Vector3d::UnitZ();

	double pix_ratio_w = space_width / (pixels_w - 1);
	double pix_ratio_h = space_height / (pixels_h - 1);

	double offset_w = 0.5 * space_width;
	double offset_h = 0.5 * space_height;

	Eigen::Vector3d ray_dir;
	double intensity;

	size_t hit_count = 0;


	std::cout << "Ray Extreme Min: " << (dir + r * (-offset_w) + u * (-offset_h)).transpose() << std::endl;
	std::cout << "Ray Extreme Max: " << (dir + r * (offset_w) + u * (offset_h)).transpose() << std::endl;

	for (size_t w = 0; w < pixels_w; ++w)
	{
		for (size_t h = 0; h < pixels_h; ++h)
		{
			ray_dir = (dir + r * (w * pix_ratio_w - offset_w) + u * (h * pix_ratio_h - offset_h)).normalized();

			auto line_intersect = LinecastTriangle(&translation, &ray_dir, &p0, &p1, &p2);

			if (line_intersect.first)
			{
				++hit_count;

				intensity = 255.0 / (1.0 + colour_shift * line_intersect.second * line_intersect.second);

				raycast_canvas(w, pixels_h - 1 - h, 0, 0) = intensity;
				raycast_canvas(w, pixels_h - 1 - h, 0, 1) = intensity;
				raycast_canvas(w, pixels_h - 1 - h, 0, 2) = intensity;
			}
		}
	}

	std::cout << "Intersect count: " << hit_count << "/" << (pixels_w * pixels_h) << std::endl;

	raycast_canvas.save_png(output_texture_name.c_str());
}

void RaycastSuite::TestSingleCast()
{
	Eigen::Vector3d ray_dir;
	std::pair<bool, double> line_intersect;
	
	ray_dir = transform * Eigen::Vector3d::UnitZ();
	line_intersect = LinecastTriangle(&translation, &ray_dir, &p0, &p1, &p2);
	std::cout << "HIT: " << line_intersect.first << ", DIST: " << line_intersect.second << std::endl;

	ray_dir = transform * Eigen::Vector3d::UnitX();
	line_intersect = LinecastTriangle(&translation, &ray_dir, &p0, &p1, &p2);
	std::cout << "HIT: " << line_intersect.first << ", DIST: " << line_intersect.second << std::endl;
}

bool RaycastSuite::InitializeSDF()
{
	GridDataStruct gds;

	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	if (!sdf.InitializeGrid(gds))
	{
		std::cout << "COULDN'T INITIALIZE GRID!" << std::endl;
		return false;
	}

	if (!mesh.ReadOBJ(input_mesh_name))
	{
		std::cout << "ERROR: mesh could not be loaded: " << input_mesh_name << std::endl;
		return false;
	}

	return true;
}

void RaycastSuite::TestMesh()
{
	if (!InitializeSDF())
	{
		return;
	}

	sdf.CastMeshNaiveRaycast(mesh);

	//sdf.PrintCrossSectionOfDoubleGrid(80);

	auto new_mesh = sdf.ExtractMeshQuantized();

	new_mesh->WriteOBJ(output_mesh_name);
}


void RaycastSuite::run(int argc, char** argv)
{
	//InitializeTransform();

	//TestMultiCast();

	//TestSingleCast();

	TestMesh();
}


