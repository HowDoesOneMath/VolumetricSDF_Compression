#pragma once

#include "VV_Mesh.h"

class MeshPartition
{
public:
	struct Partition
	{
		double area = 0;
		double volume = 0;
		std::vector<size_t> triangle_indices;
		std::vector<size_t> vertex_indices;
		size_t rightmost_point = 0;

		Eigen::Vector3d upper_bound = Eigen::Vector3d(-DBL_MAX, -DBL_MAX, -DBL_MAX);
		Eigen::Vector3d lower_bound = Eigen::Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
		Eigen::Vector3d centerpoint = Eigen::Vector3d::Zero();
	};

	std::vector<Partition> partitions;
	std::vector<int> vertex_partition_map;

	Eigen::Vector3d absolute_upper_bound = Eigen::Vector3d(-DBL_MAX, -DBL_MAX, -DBL_MAX);
	Eigen::Vector3d absolute_lower_bound = Eigen::Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	Eigen::Vector3d absolute_centerpoint = Eigen::Vector3d::Zero();

	void CreateUnionFindPartitions(VV_Mesh& mesh);

	void LightweightUnionFindPartitions(VV_Mesh& mesh);

	std::shared_ptr<std::vector<std::pair<Eigen::Vector3d, std::vector<size_t>>>> CreateNormalPartitions(VV_Mesh& mesh, double maximum_normal_difference);

	void NegateElementsOfPartition(VV_Mesh &mesh, size_t index);

	size_t NegateInsignificantPartitions(VV_Mesh& mesh, double significance_value);

	size_t NegateInteriorShells(VV_Mesh& mesh);

	bool PartitionOverlaps(int larger, int smaller);

	void ClearPartitions();
};