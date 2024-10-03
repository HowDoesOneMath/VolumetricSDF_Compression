#include "UnionFindSuite.h"

void UnionFindSuite::RunTests()
{
	test_mesh.ReadOBJ(to_load);

	TestUnionFind();
}

void UnionFindSuite::TestUnionFind()
{
	auto point_unionfind_begin = std::chrono::high_resolution_clock::now();
	mp_unionfind.CreateUnionFindPartitions(test_mesh);
	auto point_unionfind_done = std::chrono::high_resolution_clock::now();

	std::cout << "UnionFind: " << (point_unionfind_done - point_unionfind_begin) << std::endl;

}

void UnionFindSuite::run(int argc, char** argv)
{
	RunTests();
}
