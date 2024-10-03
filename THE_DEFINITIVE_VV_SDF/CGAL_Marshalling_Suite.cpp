#include "CGAL_Marshalling_Suite.h"

void CGAL_Marshalling_Suite::TestEntireSequence(std::string sequence_folder)
{
	if (!sf.FindFiles(sequence_folder, mesh_sf))
	{
		std::cout << "ERROR: problem reading from input folder! (" << sequence_folder << ")" << std::endl;
		return;
	}

	for (int i = 0; i < sf.files[mesh_sf.key].size(); ++i)
	{
		std::cout << "Testing " << i << "... " << std::endl;

		test_mesh.ReadOBJ(sf.files[mesh_sf.key][i]);

		auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(test_mesh.vertices);

		auto index_remapping = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, test_mesh.vertices);
	}
}

void CGAL_Marshalling_Suite::run(int argc, char** argv)
{
	TestEntireSequence(input_sequence);
}
