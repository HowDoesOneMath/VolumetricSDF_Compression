#include "DisplacementWaveletSuite.h"

void DisplacementWaveletSuite::WaveletPackAndUnpackDisplacements(std::vector<Eigen::Vector3d>& displacements, std::string tex_name)
{
	Eigen::Vector3d max_disp = -Eigen::Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	Eigen::Vector3d min_disp = Eigen::Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);

	for (size_t i = 0; i < displacements.size(); ++i)
	{
		max_disp = max_disp.cwiseMax(displacements[i]);
		min_disp = min_disp.cwiseMin(displacements[i]);
	}

	std::cout << "MIN: " << min_disp.transpose() << ", MAX: " << max_disp.transpose() << std::endl;

	to_pack.assign(wavelet_coeff_jpg_dim, wavelet_coeff_jpg_dim, 1, 3, 0);
	dp.FillImageBlocksRaster(to_pack, wavelet_coeff_block_size, displacements, 0, max_img_val, min_disp, max_disp);
	to_pack.save_jpeg(tex_name.c_str(), 100);

	to_pack.assign(tex_name.c_str());

	dp.RetrieveImageBlocksRaster(to_pack, wavelet_coeff_block_size, displacements, 0, max_img_val, min_disp, max_disp);
}

void DisplacementWaveletSuite::SubdivideAndDisplaceMesh(int subdiv_count)
{
	to_wavelet.ReadOBJ(input_mesh_name);
	
	VV_Mesh target_sphere;
	
	target_sphere.ReadOBJ(input_target_name);
	
	auto wav_copy = to_wavelet.GetCopy();
	
	auto adj = wav_copy->SubdivideMeshAndGetAdjacencies(subdiv_count);
	
	auto sm = vcm.GenerateCGAL_MeshFromAttribute(target_sphere.vertices);
	auto ind = vcm.CGAL_To_VV_IndexMap(*sm, target_sphere.vertices);
	auto aabb_tree = vcm.CreateAABB(*sm);
	
	auto disp = vcm.GetMeshDisplacements(*wav_copy, target_sphere, *sm, *ind, *aabb_tree);

	std::vector<Eigen::Vector3d> non_wavelet_displacements = *disp;

	auto wav_copy_copy = wav_copy->GetCopy();

	for (size_t i = 0; i < disp->size(); ++i)
	{
		wav_copy->vertices.elements[i] += (*disp)[i];
	}

	for (int i = adj->size() - 1; i >= 0; --i)
	{
		size_t starting_index = (*adj)[i].first;

		for (size_t j = 0; j < (*adj)[i].second.size(); ++j)
		{
			size_t anchor_0 = std::get<0>((*adj)[i].second[j]);
			size_t anchor_1 = std::get<1>((*adj)[i].second[j]);

			(*disp)[j + starting_index] -= 0.5 * ((*disp)[anchor_0] + (*disp)[anchor_1]);
		}
	}

	WaveletPackAndUnpackDisplacements(*disp, output_tex_wav_name);
	WaveletPackAndUnpackDisplacements(non_wavelet_displacements, output_tex_name);

	for (int i = 0; i < adj->size(); ++i)
	{
		size_t starting_index = (*adj)[i].first;

		for (size_t j = 0; j < (*adj)[i].second.size(); ++j)
		{
			size_t anchor_0 = std::get<0>((*adj)[i].second[j]);
			size_t anchor_1 = std::get<1>((*adj)[i].second[j]);

			(*disp)[j + starting_index] += 0.5 * ((*disp)[anchor_0] + (*disp)[anchor_1]);
		}
	}
	
	for (size_t i = 0; i < disp->size(); ++i)
	{
		wav_copy_copy->vertices.elements[i] += (*disp)[i];
	}

	wav_copy_copy->ConcatenateMesh(target_sphere, Eigen::Vector3d(2.1, 0, 0));
	wav_copy->ConcatenateMesh(*wav_copy_copy, Eigen::Vector3d(2.1, 0, 0));
	to_wavelet.ConcatenateMesh(*wav_copy, Eigen::Vector3d(2.1, 0, 0));
	
	to_wavelet.WriteOBJ(output_mesh_name);
}

void DisplacementWaveletSuite::run(int argc, char** argv)
{
	SubdivideAndDisplaceMesh(4);
}
