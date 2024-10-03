#pragma once

#include "TestSuite.h"

#include "VV_CGAL_Marshaller.h"

#include "VV_Mesh.h"

class CGAL_RepairSuite : public TestSuite
{
	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;

	std::string input_mesh_name = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	std::string output_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/Ab-2punch_CGAL_REPAIR.obj";

	VV_Mesh test_mesh;

	Eigen::Vector3d offset = Eigen::Vector3d(0.8, 0, 0);

	void RepairTestMesh();

public:
	void run(int argc, char** argv);
};