#pragma once

#include "TestSuite.h"

#include"VV_Mesh.h"
#include "VV_TSDF.h"
#include "TextureRemapper.h"

#include "MeshPartition.h"

#include "VV_CGAL_Marshaller.h"
#include <chrono>

#include <CImg.h>

class AttributeMapSuite : public TestSuite
{
	std::string input_mesh = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	std::string input_texture = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/tex_AB-2punch_0000001.jpg";

	std::string output_mesh_vsmc = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-2punch_RECONSTR.obj";
	std::string output_texture_vsmc = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-2punch_RECONSTR.png";

	std::string output_mesh_sdf = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-2punch_RECONSTR_SDF.obj";
	std::string output_texture_sdf = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-2punch_RECONSTR_SDF.png";

	double decimation_ratio = 0.1;

	size_t attribute_map_size = 2048;
	float gutter = 1.1f;

	double uv_remap_epsilon = 0.0001;
	int subdivision_count = 2;

	cimg_library::CImg<unsigned char> original_image;
	cimg_library::CImg<unsigned char> reconstructed_image;

	size_t grid_width_voxels = 128; 
	double grid_width_meters = 1.6;
	Eigen::Vector3d grid_center = Eigen::Vector3d(0.0, 0.0, 0.0);

	size_t estimated_uv_partitions = 160;
	double uv_partition_buffer = 0.1;
	double sdf_buffer_distance = 0.04;
	double artifact_size = 0.1;
	VV_TSDF sdf;

	MeshPartition mp;

	int padding_loops = 3;

	VV_Mesh test_mesh;
	TextureRemapper tr;
	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;
	//VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_2, Eigen::Vector2d> vcm_uv;

	void RegenerateAttributeMapVSMC();

	void RegenerateAttributeMapSDF();

	void SingleTestDecimation();
	void ReadJPGTest();
public:
	void run(int argc, char** argv);
};