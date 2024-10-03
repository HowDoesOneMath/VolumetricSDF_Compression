#include "MeshToPointCloudSuite.h"

void MeshToPointCloudSuite::run(int argc, char** argv)
{
	to_test.ReadOBJ(input_mesh_name);

	to_test.normals.Clear();
	to_test.uvs.Clear();

	auto example_mesh = to_test.GetCopy();

	VV_Mesh pcg_casts;
	VV_Mesh single_cast;

	auto points = pcg.GeneratePointsFromMesh(to_test, generation_spacing, point_offset);

	int next_axis;

	PointCloudGenerator::PCG_Point* current_p;

	Eigen::Vector3i* tri;
	Eigen::Vector3d intersect_pos;

	std::cout << "TOTAL POINTS: " << points->size() << std::endl;

	for (size_t i = 0; i < points->size(); ++i)
	{
		current_p = &(*points)[i];

		next_axis = current_p->cast_direction + 1;
		next_axis *= (next_axis < 3);

		tri = &(to_test.vertices.indices[current_p->triangle]);

		intersect_pos =
			to_test.vertices.elements[tri->x()] * current_p->barycentric_coords.x() +
			to_test.vertices.elements[tri->y()] * current_p->barycentric_coords.y() +
			to_test.vertices.elements[tri->z()] * current_p->barycentric_coords.z();

		single_cast.ApplyDebuggingCube(visual_cube_size * Eigen::Vector3d::Ones(), intersect_pos);

		pcg_casts.vertices.Concatenate(single_cast.vertices, Eigen::Vector3d::Zero());
	}

	example_mesh->ConcatenateMesh(pcg_casts, Eigen::Vector3d(0, 0, 0));

	example_mesh->ConcatenateMesh(pcg_casts, Eigen::Vector3d(concatenation_spacing, 0, 0));

	to_test.ConcatenateMesh(*example_mesh, Eigen::Vector3d(concatenation_spacing, 0, 0));
	
	to_test.WriteOBJ(output_mesh_name);
}
