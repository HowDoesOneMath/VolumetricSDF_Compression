#pragma once

#include "VV_Mesh.h"

#include "VV_CGAL_Marshaller.h"

#include "DisplacementPacker.h"

#include "TestSuite.h"

class DisplacementWaveletSuite : public TestSuite
{
	VV_Mesh to_wavelet;

	cimg_library::CImg<unsigned char> to_pack;

	DisplacementPacker<unsigned char> dp;

	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;

	std::string input_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingCube.obj";
	std::string input_target_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingSphere.obj";
	std::string output_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingCubeWAVELET.obj";
	std::string output_tex_wav_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingCubeWAVELET_APPLIED.jpg";
	std::string output_tex_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/DebuggingCubeWAVELET_NOT_APPLIED.jpg";

	size_t wavelet_coeff_jpg_dim = 64;

	size_t wavelet_coeff_block_size = 16;

	unsigned char max_img_val = 255;

	void WaveletPackAndUnpackDisplacements(std::vector<Eigen::Vector3d> &displacements, std::string tex_name);

	void SubdivideAndDisplaceMesh(int subdiv_count);

public:
	void run(int argc, char** argv);
};