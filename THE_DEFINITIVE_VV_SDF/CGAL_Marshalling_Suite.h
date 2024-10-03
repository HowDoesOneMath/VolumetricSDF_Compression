#pragma once

#include "TestSuite.h"

#include <string>
#include <iostream>

#include "SequenceFinder.h"
#include "VV_Mesh.h"
#include "VV_CGAL_Marshaller.h"

class CGAL_Marshalling_Suite : public TestSuite
{
	std::string input_sequence = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch";

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinder sf;

	VV_Mesh test_mesh;

	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;

	void TestEntireSequence(std::string sequence_folder);
public:
	void run(int argc, char** argv);
};