#pragma once

#include "VV_Mesh.h"
#include "AdditionalUtilities.h"

class PointCloudGenerator
{
public:
	struct PCG_Point
	{
		size_t triangle;
		Eigen::Vector3d barycentric_coords;
		int cast_direction;

		Eigen::Vector3d position;
		Eigen::Vector3d normal;

		//Eigen::Vector3d position;
		//Eigen::Vector3<unsigned char> colour;
	};

private:

public:

	std::shared_ptr<std::vector<PCG_Point>> GeneratePointsFromMesh(VV_Mesh& mesh, double spacing, Eigen::Vector3d offset);
};