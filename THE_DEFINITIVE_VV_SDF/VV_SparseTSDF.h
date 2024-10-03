#pragma once


#include <vector>
#include <fstream>

#include <fftw3.h>

#include "VV_Mesh.h"
#include "MeshPartition.h"
#include "VV_SaveFileBuffer.h"

#include "GridDataStruct.h"
//#include "MarchingCubesSolver.h"

#include <queue>
#include <unordered_map>

#include "BasicGeometry.h"

class VV_SparseTSDF
{
	std::unordered_map<size_t, size_t> indices;
public:
	std::shared_ptr<VV_Mesh> CastUnsignedMeshToSDF(VV_Mesh& to_cast, double buffer_distance);
};