#pragma once

#include "VV_Mesh.h"

#include <vector>

struct VV_NormalGroup
{
	Eigen::Vector3d normal_direction;
	std::vector<size_t> triangles;
};

class VV_MeshNormalSeparator
{
	std::shared_ptr<std::vector<std::vector<size_t>>> CreateVertexGroups(VV_Mesh& mesh);

	std::shared_ptr<std::vector<Eigen::Vector3d>> CreateTriangleNormals(VV_Mesh& mesh, std::vector<bool>& precull_visitors);
public:
	std::shared_ptr<std::vector<VV_NormalGroup>> SeparateMesh(VV_Mesh& to_separate, double separation_threshold);
};