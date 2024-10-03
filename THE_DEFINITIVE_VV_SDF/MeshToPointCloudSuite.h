#pragma once

#include "TestSuite.h"

#include "PointCloudGenerator.h"

class MeshToPointCloudSuite : public TestSuite
{
	PointCloudGenerator pcg;

	//std::string input_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingSphere.obj";
	//std::string output_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingSphereWITH_POINTS.obj";
	
	std::string input_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/ArtifactRemovedPUNCH.obj";
	std::string output_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/PUNCH_WITH_POINTS.obj";

	double generation_spacing = 0.005;

	double concatenation_spacing = 0.7 + 2 * generation_spacing;

	double point_epsilon = 0.0000017;
	Eigen::Vector3d point_offset = Eigen::Vector3d(point_epsilon, point_epsilon, point_epsilon);

	double visual_cube_size = 0.8 * generation_spacing;

	VV_Mesh to_test;

public:
	void run(int argc, char** argv);
};