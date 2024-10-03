#include "VV_Mesh_Suite.h"

void VV_Mesh_Suite::PartitionMesh()
{
	test_mesh.ReadOBJ(partition_test);

	std::cout << "Tri Count: " << test_mesh.vertices.indices.size() << std::endl;

	//std::vector<std::pair<double, std::vector<size_t>>> partitions;

	//partition.PartitionWithAreas(test_mesh, &partitions);
	partition.CreateUnionFindPartitions(test_mesh);

	for (int i = 0; i < partition.partitions.size(); ++i)
	{
		//std::cout << "Group " << i << ": " << partitions[i].second.size() << " tris, " << partitions[i].first << " area." << std::endl;
		std::cout << "Group " << i << ": " << partition.partitions[i].triangle_indices.size() << " tris, " << partition.partitions[i].area << " area." << std::endl;
	}

	size_t culled = partition.NegateInsignificantPartitions(test_mesh, bad_mesh_threshold);
	test_mesh.ClearNegativeTriangles();

	std::cout << "CULLED: " << culled << "/" << partition.partitions.size() << std::endl;

	test_mesh.ClearUnreferencedElements();

	test_mesh.WriteOBJ(partition_output);
}

void VV_Mesh_Suite::ReadWrite()
{
	test_mesh.ReadOBJ(input_file);

	test_mesh.WriteOBJ(output_file);
}

void VV_Mesh_Suite::run(int argc, char** argv)
{
	//ReadWrite();
	PartitionMesh();
}
