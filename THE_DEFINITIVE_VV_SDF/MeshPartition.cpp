#include "MeshPartition.h"


void MeshPartition::CreateUnionFindPartitions(VV_Mesh& mesh)
{
	ClearPartitions();

	size_t vert_count = mesh.vertices.elements.size();
	size_t ind_count = mesh.vertices.indices.size();

	Eigen::Vector3i* tris = mesh.vertices.indices.data();
	Eigen::Vector3d* verts = mesh.vertices.elements.data();

	size_t starting_node;
	size_t partition_count = 0;

	size_t parent_0;
	size_t parent_1;
	size_t parent_2;

	std::vector<size_t> joints(vert_count);
	std::vector<size_t> partition_index(vert_count, SIZE_MAX);
	std::vector<size_t> backwards_links(vert_count);
	std::vector<size_t> child_count(vert_count, 0);

	vertex_partition_map.resize(vert_count, 0);

	Eigen::Vector3d* v0;
	Eigen::Vector3d* v1;
	Eigen::Vector3d* v2;

	Eigen::Vector3d v01;
	Eigen::Vector3d v02;

	Eigen::Vector3d c_01_02;

	for (size_t i = 0; i < vert_count; ++i)
	{
		joints[i] = i;
	
		absolute_upper_bound = absolute_upper_bound.cwiseMax(verts[i]);
		absolute_lower_bound = absolute_lower_bound.cwiseMin(verts[i]);
	}
	
	absolute_centerpoint = (absolute_upper_bound + absolute_lower_bound) * 0.5;

	for (size_t i = 0; i < ind_count; ++i)
	{
		parent_0 = tris[i].x();
		while (joints[parent_0] != parent_0)
		{
			//backwards_links[joints[parent_0]] = parent_0;
			parent_0 = joints[parent_0];
		}
		//starting_node = parent_0;
		//while (starting_node != tris[i].x())
		//{
		//	starting_node = backwards_links[starting_node];
		//	joints[starting_node] = parent_0;
		//}

		parent_1 = tris[i].y();
		while (joints[parent_1] != parent_1)
		{
			//backwards_links[joints[parent_1]] = parent_1;
			parent_1 = joints[parent_1];
		}
		//starting_node = parent_1;
		//while (starting_node != tris[i].y())
		//{
		//	starting_node = backwards_links[starting_node];
		//	joints[starting_node] = parent_0;
		//}

		parent_2 = tris[i].z();
		while (joints[parent_2] != parent_2)
		{
			//backwards_links[joints[parent_2]] = parent_2;
			parent_2 = joints[parent_2];
		}
		//starting_node = parent_2;
		//while (starting_node != tris[i].z())
		//{
		//	starting_node = backwards_links[starting_node];
		//	joints[starting_node] = parent_0;
		//}

		joints[parent_1] = parent_0;

		joints[parent_2] = parent_0;
	}

	for (size_t i = 0; i < joints.size(); ++i)
	{
		parent_0 = i;
		while (joints[parent_0] != parent_0)
		{
			backwards_links[joints[parent_0]] = parent_0;
			parent_0 = joints[parent_0];
		}
	
		parent_2 = parent_0;
	
		//partition_index[parent_2] = partition_count * (i == parent_2);
		//partition_count += (i == parent_2);
		//child_count[parent_2] += (i == parent_2);
	
		while (parent_0 != i)
		{
			parent_0 = backwards_links[parent_0];
			joints[parent_0] = parent_2;
			//++child_count[parent_2];
		}
	}

	//partitions.resize(partition_count);
	//for (size_t i = 0; i < partitions.size(); ++i)
	//{
	//	partitions[i].resize(child_count[])
	//}

	//for (size_t i = 0; i < ind_count; ++i)
	//{
	//	++child_count[joints[tris[i][0]]];
	//}

	for (size_t i = 0; i < ind_count; ++i)
	{
		parent_0 = joints[tris[i][0]];
		//parent_1 = parent_0;
		v0 = &verts[tris[i][0]];
		v1 = &verts[tris[i][1]];
		v2 = &verts[tris[i][2]];

		while (joints[parent_0] != parent_0)
		{
			//backwards_links[joints[parent_0]] = parent_0;
			parent_0 = joints[parent_0];
		}

		//parent_2 = parent_0;

		if (partition_index[parent_0] == SIZE_MAX)
		{
			partition_index[parent_0] = partitions.size();

			partitions.push_back(Partition());
			partitions.back().rightmost_point = tris[i][0];
		}

		partitions[partition_index[parent_0]].triangle_indices.push_back(i);

		for (size_t j = 0; j < 3; ++j)
		{
			if ((*v0)[j] > partitions[partition_index[parent_0]].upper_bound[j])
			{
				partitions[partition_index[parent_0]].upper_bound[j] = (*v0)[j];
			}
			if ((*v1)[j] > partitions[partition_index[parent_0]].upper_bound[j])
			{
				partitions[partition_index[parent_0]].upper_bound[j] = (*v1)[j];
			}
			if ((*v2)[j] > partitions[partition_index[parent_0]].upper_bound[j])
			{
				partitions[partition_index[parent_0]].upper_bound[j] = (*v2)[j];
			}

			if ((*v0)[j] < partitions[partition_index[parent_0]].lower_bound[j])
			{
				partitions[partition_index[parent_0]].lower_bound[j] = (*v0)[j];
			}
			if ((*v1)[j] < partitions[partition_index[parent_0]].lower_bound[j])
			{
				partitions[partition_index[parent_0]].lower_bound[j] = (*v1)[j];
			}
			if ((*v2)[j] < partitions[partition_index[parent_0]].lower_bound[j])
			{
				partitions[partition_index[parent_0]].lower_bound[j] = (*v2)[j];
			}
		}

		if (v0->x() > verts[partitions[partition_index[parent_0]].rightmost_point].x())
			partitions[partition_index[parent_0]].rightmost_point = tris[i][0];
		if (v1->x() > verts[partitions[partition_index[parent_0]].rightmost_point].x())
			partitions[partition_index[parent_0]].rightmost_point = tris[i][1];
		if (v2->x() > verts[partitions[partition_index[parent_0]].rightmost_point].x())
			partitions[partition_index[parent_0]].rightmost_point = tris[i][2];

		v01 = *v0 - *v1;
		v02 = *v0 - *v2;

		c_01_02.x() = v01.y() * v02.z() - v01.z() * v02.y();
		c_01_02.y() = v01.z() * v02.x() - v01.x() * v02.z();
		c_01_02.z() = v01.x() * v02.y() - v01.y() * v02.x();

		partitions[partition_index[parent_0]].area += c_01_02.norm();
		partitions[partition_index[parent_0]].volume += c_01_02.dot(*v0 - absolute_centerpoint);

		//while (parent_0 != parent_1)
		//{
		//	parent_0 = backwards_links[parent_0];
		//	joints[parent_0] = parent_2;
		//}
	}

	for (size_t i = 0; i < vert_count; ++i)
	{
		partitions[partition_index[joints[i]]].vertex_indices.push_back(i);
		vertex_partition_map[i] = partition_index[joints[i]];
	}

	for (size_t i = 0; i < partitions.size(); ++i)
	{
		partitions[i].area *= 0.5;
		partitions[i].volume /= 6;

		partitions[i].centerpoint = (partitions[i].upper_bound + partitions[i].lower_bound) * 0.5;
	}
}

void MeshPartition::LightweightUnionFindPartitions(VV_Mesh& mesh)
{
	ClearPartitions();

	size_t vert_count = mesh.vertices.elements.size();
	size_t ind_count = mesh.vertices.indices.size();

	Eigen::Vector3i* tris = mesh.vertices.indices.data();

	size_t starting_node;
	size_t partition_count = 0;

	size_t parent_0;
	size_t parent_1;
	size_t parent_2;

	std::vector<size_t> joints(vert_count);
	std::vector<size_t> partition_index(vert_count, SIZE_MAX);
	std::vector<size_t> backwards_links(vert_count);

	vertex_partition_map.resize(vert_count, 0);

	for (size_t i = 0; i < vert_count; ++i)
	{
		joints[i] = i;
	}

	for (size_t i = 0; i < ind_count; ++i)
	{
		parent_0 = tris[i].x();
		while (joints[parent_0] != parent_0)
		{
			parent_0 = joints[parent_0];
		}

		parent_1 = tris[i].y();
		while (joints[parent_1] != parent_1)
		{
			parent_1 = joints[parent_1];
		}

		parent_2 = tris[i].z();
		while (joints[parent_2] != parent_2)
		{
			parent_2 = joints[parent_2];
		}

		joints[parent_1] = parent_0;

		joints[parent_2] = parent_0;
	}

	for (size_t i = 0; i < joints.size(); ++i)
	{
		parent_0 = i;
		while (joints[parent_0] != parent_0)
		{
			backwards_links[joints[parent_0]] = parent_0;
			parent_0 = joints[parent_0];
		}

		parent_2 = parent_0;

		while (parent_0 != i)
		{
			parent_0 = backwards_links[parent_0];
			joints[parent_0] = parent_2;
		}
	}

	for (size_t i = 0; i < ind_count; ++i)
	{
		parent_0 = joints[tris[i][0]];

		while (joints[parent_0] != parent_0)
		{
			parent_0 = joints[parent_0];
		}

		if (partition_index[parent_0] == SIZE_MAX)
		{
			partition_index[parent_0] = partitions.size();

			partitions.push_back(Partition());
		}

		partitions[partition_index[parent_0]].triangle_indices.push_back(i);
	}

	for (size_t i = 0; i < vert_count; ++i)
	{
		vertex_partition_map[i] = partition_index[joints[i]];
	}
}

std::shared_ptr<std::vector<std::pair<Eigen::Vector3d, std::vector<size_t>>>> MeshPartition::CreateNormalPartitions(VV_Mesh& mesh, double maximum_normal_difference)
{
	auto to_return = std::make_shared<std::vector<std::pair<Eigen::Vector3d, std::vector<size_t>>>>();

	Eigen::Vector3i* tri;
	Eigen::Vector3d* v0;
	Eigen::Vector3d* v1;
	Eigen::Vector3d* v2;

	Eigen::Vector3d norm_direction;
	double norm_similarity;
	int chosen_partition;
	double strongest_similarity;

	for (size_t i = 0; i < mesh.vertices.indices.size(); ++i)
	{
		tri = &mesh.vertices.indices[i];
		v0 = &mesh.vertices.elements[tri->x()];
		v1 = &mesh.vertices.elements[tri->y()];
		v2 = &mesh.vertices.elements[tri->z()];

		norm_direction = CrossProduct(*v1 - *v0, *v2 - *v0).normalized();

		chosen_partition = -1;
		strongest_similarity = -DBL_MAX;

		for (size_t j = 0; j < to_return->size(); ++j)
		{
			norm_similarity = norm_direction.dot((*to_return)[j].first);

			if (norm_similarity > strongest_similarity)
			{
				chosen_partition = j;
				strongest_similarity = norm_similarity;
			}
		}

		if ((strongest_similarity > maximum_normal_difference) && (chosen_partition >= 0))
		{
			(*to_return)[chosen_partition].second.push_back(i);
			continue;
		}

		to_return->push_back(std::make_pair(norm_direction, std::vector<size_t>()));
		to_return->back().second.push_back(i);
	}

	return to_return;
}

void MeshPartition::NegateElementsOfPartition(VV_Mesh& mesh, size_t index)
{
	for (int j = 0; j < partitions[index].triangle_indices.size(); ++j)
	{
		mesh.NegateTriangle(partitions[index].triangle_indices[j]);
	}

	partitions[index].triangle_indices.clear();
}

size_t MeshPartition::NegateInsignificantPartitions(VV_Mesh& mesh, double significance_value)
{
	size_t culled = 0;

	for (int i = 0; i < partitions.size(); ++i)
	{
		if (partitions[i].area < significance_value)
		{
			++culled;

			NegateElementsOfPartition(mesh, i);
		}
	}

	return culled;
}

size_t MeshPartition::NegateInteriorShells(VV_Mesh& mesh)
{
	size_t culled = 0;

	for (int i = 0; i < partitions.size(); ++i)
	{
		if (partitions[i].volume <= 0)
		{
			++culled;

			NegateElementsOfPartition(mesh, i);
		}
	}

	return culled;
}

bool MeshPartition::PartitionOverlaps(int larger, int smaller)
{
	if ((partitions[larger].triangle_indices.size() <= 0) || (partitions[smaller].triangle_indices.size() <= 0))
	{
		return false;
	}

	if ((partitions[larger].upper_bound.x() >= partitions[smaller].upper_bound.x())
		&& (partitions[larger].upper_bound.y() >= partitions[smaller].upper_bound.y())
		&& (partitions[larger].upper_bound.z() >= partitions[smaller].upper_bound.z())
		&& (partitions[larger].lower_bound.x() <= partitions[smaller].lower_bound.x())
		&& (partitions[larger].lower_bound.y() <= partitions[smaller].lower_bound.y())
		&& (partitions[larger].lower_bound.z() <= partitions[smaller].lower_bound.z())
		)
	{
		return true;
	}

	return false;
}

void MeshPartition::ClearPartitions()
{
	absolute_centerpoint = Eigen::Vector3d::Zero();
	absolute_upper_bound = Eigen::Vector3d(-DBL_MAX, -DBL_MAX, -DBL_MAX);
	absolute_lower_bound = Eigen::Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	partitions.clear();
	//areas.clear();
	//volumes.clear();
	//bounding_boxes.clear();
	//partitions.clear();
	//rightmost_point.clear();
	vertex_partition_map.clear();
}
