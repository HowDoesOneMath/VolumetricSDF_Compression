#include "MC_TestSuite.h"

void MC_TestSuite::run(int argc, char** argv)
{
	GridDataStruct gds(
		grid_len, grid_len, grid_len,
		0, 0, 0,
		grid_span / (grid_len - 1)
	);

	if (!tsdf.InitializeGrid(gds))
	{
		std::cout << "ERROR! Grid couldn't initialize!" << std::endl;
		return;
	}

	tsdf.MakeDebuggingSphere(Eigen::Vector3d(0, 0, 0), 1.01, mesh_name);
}
