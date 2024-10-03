#include "SubdivisionSuite.h"

void SubdivisionSuite::SubdivideTestMesh()
{
	to_subdivide.ReadOBJ(mesh_input);

	to_subdivide.DebugGeneralStats();

	//auto adj = to_subdivide.SubdivideMeshAndGetAdjacencies(iterations);
	to_subdivide.SubdivideMesh(iterations);

	to_subdivide.DebugGeneralStats();

	to_subdivide.WriteOBJ(mesh_output);
}

void SubdivisionSuite::run(int argc, char** argv)
{
	SubdivideTestMesh();
}
