#pragma once

#include "TestSuite.h"

#include "VV_Mesh.h"
#include "VV_CGAL_Marshaller.h"
#include "MortonOrderer.h"
#include "TextureRemapper.h"
#include "RandomWrapper.h"

#include "VSMC_Compressor.h"

#include "SequenceFinder.h"
#include "VV_SaveFileBuffer.h"

#include <CImg.h>

#include <iostream>

class VSMC_TestSuite : public TestSuite
{
	std::string test_mesh_name = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/AB-2punch_0000001.obj";
	std::string test_texture_name = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch/tex_AB-2punch_0000001.jpg";

	std::string output_base_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_0000001_VSMC_DECIM.obj";
	std::string output_displaced_mesh_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_0000001_VSMC_SUBDIV.obj";
	std::string output_texture_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_0000001_VSMC_TEX.png";
	std::string output_displacement_name = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ArbitraryTestMeshes/AB-Punch_0000001_VSMC_DISP.png";

	// For use with GarlandHeckbert_triangle_policies - see VV_CGAL_Marshaller
	//std::string test_sequence = "D:/_VV_DATASETS_TRIMMED/AB-2punch";
	//std::string test_sequence = "D:/_VV_DATASETS_TRIMMED/AB-dodgeleft";
	//std::string test_sequence = "D:/_VV_DATASETS_TRIMMED/AB-death";
	std::string test_sequence = "E:/_VV_DATA/_VV_DATASETS_TRIMMED/SIR_FREDRICK";
	//std::string sequence_file_identifier = "/AB-2PUNCH";
	//std::string sequence_file_identifier = "/AB-DODGE";
	//std::string sequence_file_identifier = "/AB-DEATH";
	std::string sequence_file_identifier = "/SIR_FREDRICK";

	//size_t atlas_size = 1024;
	//size_t atlas_size = 2048;
	size_t atlas_size = 4096;

	//size_t displacement_texture_size = 128;
	//size_t displacement_texture_size = 256;
	size_t displacement_texture_size = 512;
	//size_t displacement_texture_size = 1024;
	size_t displacement_block_size = 16;

	int draco_compression_level = 0;
	//int draco_compression_level = 7;
	//int draco_compression_level = 10;

	//int decim_int = 4;
	//int subdiv_loops = 1;
	//int decim_int = 10;
	//int subdiv_loops = 2;
	//int decim_int = 16;
	//int subdiv_loops = 2;
	//int decim_int = 40;
	//int subdiv_loops = 3;
	int decim_int = 64;
	int subdiv_loops = 3;

	double decimation_ratio = 1.0 / decim_int;

	std::string draco_level = "/DRACO_COMPRESSION_" + std::to_string(draco_compression_level);
	std::string compression_details = "/DECIM" + std::to_string(decim_int) + "LOOP" + std::to_string(subdiv_loops);

	std::string compressed_sequence_folder = "E:/_VV_DATA/_COMPRESSIONS/_VSMC" + draco_level + compression_details + sequence_file_identifier;
	std::string reconstructed_sequence_folder = "E:/_VV_DATA/_RECONSTRUCTIONS/_VSMC" + draco_level + compression_details + sequence_file_identifier;

	std::string compress_file_output = compressed_sequence_folder + "/CompressedSequence.vsmc";
	std::string displacement_tag = compressed_sequence_folder + "/Displacement";

	std::string reconstructed_mesh_tag = reconstructed_sequence_folder + "/FRAME";
	std::string texture_tag = reconstructed_mesh_tag;

	VSMC_Compressor vsmc_comp;

	MortonOrderer mo;
	TextureRemapper tr;
	RandomWrapper rw;

	VV_SaveFileBuffer sfb;
	SequenceFinder sf;
	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinderDetails texture_sf = SequenceFinderDetails("Texture", ".jpg");
	SequenceFinderDetails displacement_sf = SequenceFinderDetails("Displacement", ".jpg");

	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;


	//size_t end_frame = SIZE_MAX;
	size_t end_frame = 3;
	size_t beginning_frame = 0;

	double gutter_size = 1.1;
	double uv_epsilon = 0.0001;


	int push_pull_kernel_size = 5;
	int push_pull_kernel_scale = 2.0;

	template<typename T>
	void FillImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, std::vector<Eigen::Vector3d>& data,
		T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val);

	template<typename T>
	std::shared_ptr<std::vector<Eigen::Vector3d>> UnpackImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size,
		T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val);

	void CompressSingularMesh();

	void TestGetAdjacencies();

	void CompressSequence(std::string root_folder, std::string save_name);
public:
	void run(int argc, char** argv);
};

template<typename T>
inline void VSMC_TestSuite::FillImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, std::vector<Eigen::Vector3d>& data,
	T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val)
{
	std::cout << "MIN: " << min_data_val.transpose() << std::endl;
	std::cout << "MAX: " << max_data_val.transpose() << std::endl;

	Eigen::Vector3d span_data = max_data_val - min_data_val;

	double span_img = (double)max_img_val - (double)min_img_val;

	size_t image_coords[2];
	size_t morton_x;
	size_t morton_y;

	size_t block_size_sqr = block_size * block_size;

	size_t loc = 0;

	for (size_t x = 0; x < image.width(); x += block_size)
	{
		for (size_t y = 0; y < image.height(); y += block_size)
		{
			//std::cout << "BLOCK " << x << ", " << y << ": " << std::endl;

			for (size_t i = 0; i < block_size_sqr; ++i, ++loc)
			{
				if (loc >= data.size())
				{
					return;
				}

				mo.GetNormalOrder(image_coords, 2, i);

				morton_y = image_coords[1] + y;
				morton_x = image_coords[0] + x;

				for (size_t c = 0; c < 3; ++c)
				{
					image(morton_x, morton_y, 0, c) = (T)(((data[loc][c] - min_data_val[c]) / span_data[c]) * span_img) + min_img_val;

					//if (rw.InRange(0, 10000) == 0)
					//{
					//	std::cout << "RANDOMLY POLLED " << loc << " (" << morton_x << ", " << morton_y << ") channel " << c << ": " 
					//		<< (int)image(morton_x, morton_y, 0, c) << std::endl;
					//}
				}
			}
		}
	}
}

template<typename T>
inline std::shared_ptr<std::vector<Eigen::Vector3d>> VSMC_TestSuite::UnpackImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val)
{
	auto to_return = std::make_shared<std::vector<Eigen::Vector3d>>();



	return to_return;
}
