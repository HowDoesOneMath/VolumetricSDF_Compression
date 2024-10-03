#include "SequenceFinderSuite.h"

void SequenceFinderSuite::FindTestSequence()
{
	std::cout << "Finding meshes... " << std::endl;

	std::vector<SequenceFinderDetails> details = { mesh_sf, tex_png_sf, tex_jpg_sf };

	if (!sf.FindFiles(input_folder, details))
	{
		return;
	}

	std::cout << sf.files[sf_mesh_key].size() << std::endl;
	//std::cout << sf.files[sf_texture_key].size() << std::endl;

	if (sf.files[sf_mesh_key].size() != sf.files[sf_texture_key].size())
	{
		std::cout << "Mismatch of mesh and texture file count!\nMesh: " << sf.files[sf_mesh_key].size() << "\nTexture: " << sf.files[sf_texture_key].size() << std::endl;
	}

	for (int i = 0; i < sf.files[sf_mesh_key].size(); ++i)
	{
		std::cout << i << ": " << sf.files[sf_mesh_key][i] << ", " << sf.files[sf_texture_key][i] << std::endl;
	}
}

void SequenceFinderSuite::run(int argc, char** argv)
{
	FindTestSequence();
}
