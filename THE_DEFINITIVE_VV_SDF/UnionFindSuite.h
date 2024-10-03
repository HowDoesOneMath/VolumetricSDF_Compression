#pragma once

#include <string>
#include "TestSuite.h"

#include "MeshPartition.h"
#include "UnionFindNode.h"
#include "VV_Mesh.h"

#include <iostream>
#include <chrono>

class UnionFindSuite : public TestSuite
{
	std::string to_load = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";

	VV_Mesh test_mesh;
	MeshPartition mp_unionfind;
	MeshPartition mp_partition;

	//std::vector<UnionFindNode> nodes_triangles;
	std::vector<UnionFindNode> nodes_vertices;
	void RunTests();

	void TestUnionFind();

public:
	void run(int argc, char** argv);
};