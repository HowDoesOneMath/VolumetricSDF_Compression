#include "UVAtlasSuite.h"

void UVAtlasSuite::RecreateUVs()
{
	vv_mesh.ReadOBJ(input_mesh);

	//vv_mesh.GenerateNewUVs();

	vv_mesh.WriteOBJ(output_mesh);
}

void UVAtlasSuite::run(int argc, char** argv)
{
	RecreateUVs();
}
