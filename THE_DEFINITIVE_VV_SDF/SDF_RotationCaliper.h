#pragma once

#include "VV_Mesh.h"
#include "VV_TSDF.h"

#include "MeshPartition.h"
#include "RectanglePacker.h"
#include "ConvexHullCreator.h"

#include <Eigen/Geometry>

#include <set>
#include <unordered_set>
#include <unordered_map>

#include <CImg.h>
#include "CImgAdditionalDrawingFunctionality.h"

class SDF_RotationCaliper
{
	ConvexHullCreator chc;
	MeshPartition mp;
	RectanglePacker<double> rp;

	void DebugDisplayPatch(VV_Mesh &mesh, std::vector<size_t>& hull, std::pair<size_t, size_t> least_box, size_t display_size = 512);

	void DebugDisplayPatchLinear(VV_Mesh& mesh, std::vector<size_t>& hull, Eigen::Vector4i least_box, size_t display_size = 512);

public:
	std::shared_ptr<std::vector<Eigen::Vector2d>> GetHullEdges(VV_Mesh& submesh, std::vector<size_t>& hull);

	std::shared_ptr<std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> GetAntipodalPoints(std::vector<Eigen::Vector2d>& edges);

	std::pair<Eigen::Matrix2d, std::pair<size_t, size_t>> GetOptimalRotation(VV_Mesh& submesh, std::vector<size_t>& hull);

	std::pair<Eigen::Matrix2d, Eigen::Vector4i> GetOptimalRotationLinear(VV_Mesh& submesh, std::vector<size_t>& hull);

	std::pair<Eigen::Vector2d, Eigen::Vector2d> RotateUVs(VV_Mesh& mc_mesh);

	std::shared_ptr<std::vector<std::pair<Eigen::Vector3d, std::vector<size_t>>>> GetGroupsByNormal(VV_Mesh& mc, double minimum_normal_similarity);

	void ConstructRotationCaliperAtlas(VV_Mesh& mc_mesh, Eigen::Vector2d spot_start, Eigen::Vector2d spot_dims, 
		double minimum_normal_similarity, double pad_amount);


};