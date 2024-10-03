#pragma once

#include <iostream>

#include "VV_Mesh.h"
#include "VV_CGAL_Marshaller.h"

#include "SequenceFinder.h"
#include "VV_SaveFileBuffer.h"

#include "MeshPartition.h"
#include "MortonOrderer.h"
#include "WaveletTransformer.h"
#include "DracoCompressor.h"
#include "TextureRemapper.h"

#include "DisplacementPacker.h"

#define USE_DOUBLE_DISPLACEMENTS 0
//#define USE_DOUBLE_DISPLACEMENTS 1

class VSMC_Compressor
{
	std::shared_ptr<VV_Mesh> last_intra;

	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;

#if USE_DOUBLE_DISPLACEMENTS
	DisplacementPacker<double> dp;
#else
	DisplacementPacker<unsigned char> dp;
#endif

	WaveletTransformer wave_trans;
	TextureRemapper tr;

	MortonOrderer mo;

	MeshPartition mp;

	SequenceFinder sf;
	VV_SaveFileBuffer sfb;

	double dec_ratio;
	size_t width, height;
	double gutter;
	int subdivision_count;
	size_t disp_width;
	size_t b_size;

	double uv_epsilon = 0.0001;

	double mesh_maximum_artifact_size = 0.05;

#if USE_DOUBLE_DISPLACEMENTS
	double displacement_max = 255.0;
#else
	unsigned char displacement_max = 255;
#endif


	int position_quant_level = 11;
	int uv_quant_level = 10;
	int normal_quant_level = 8;

	std::vector<std::pair<draco::GeometryAttribute::Type, int>> quantization_options = {
		std::make_pair(draco::GeometryAttribute::Type::POSITION, position_quant_level),
		std::make_pair(draco::GeometryAttribute::Type::TEX_COORD, uv_quant_level),
		std::make_pair(draco::GeometryAttribute::Type::NORMAL, normal_quant_level)
	};

	int enc_speed; //= universal_draco_speed;
	int dec_speed; //= universal_draco_speed;

	DracoCompressor* dc = nullptr; //DracoCompressor(quantization_options, enc_speed, dec_speed);

	int ppk_size = 5;
	double ppk_scale = 2.0;

	unsigned int jpg_q;

	bool initialized = false;

public:
	~VSMC_Compressor() {
		if (dc != nullptr)
		{
			delete dc;
			dc = nullptr;
		}
	}

	void InitializeCompressor(double decimation_ratio, size_t output_attribute_width, size_t output_attribute_height, double gutter_amount, 
		int subdiv_count, size_t displacement_map_width, size_t displacement_block_size, int push_pull_kernel_size, double push_pull_kernel_scale,
		int draco_compression_speed, unsigned int jpg_quality = 100U);

private:
#if USE_DOUBLE_DISPLACEMENTS
	std::shared_ptr<std::pair<cimg_library::CImg<unsigned char>, cimg_library::CImg<double>>> CompressIntra(
		VV_Mesh& input_mesh, cimg_library::CImg<unsigned char>& input_texture, VV_SaveFileBuffer& sfb);
#else
	std::shared_ptr<std::pair<cimg_library::CImg<unsigned char>, cimg_library::CImg<unsigned char>>> CompressIntra(
		VV_Mesh& input_mesh, cimg_library::CImg<unsigned char> &input_texture, VV_SaveFileBuffer& sfb);
#endif

#if USE_DOUBLE_DISPLACEMENTS
	std::shared_ptr<VV_Mesh> DecompressIntra(size_t frame, std::vector<size_t>& file_locs, int subdiv_count,
		cimg_library::CImg<double> displacements, size_t block_size, VV_SaveFileBuffer& sfb);
#else
	std::shared_ptr<VV_Mesh> DecompressIntra(size_t frame, std::vector<size_t>& file_locs, int subdiv_count,
		cimg_library::CImg<unsigned char> displacements, size_t block_size, VV_SaveFileBuffer& sfb);
#endif

public:
	bool CompressSequence(std::string root_folder, SequenceFinderDetails mesh_sf, SequenceFinderDetails texture_sf, std::string output_file_name,
		std::string output_texture_tag, std::string displacement_texture_tag, size_t starting_frame = 0, size_t ending_frame = SIZE_MAX, size_t digits_per_number = 6);

	bool DecompressSequence(std::string input_file_name, std::string displacements_folder, SequenceFinderDetails displacement_texture_sf, 
		std::string output_mesh_tag, size_t digits_per_number = 6);

private:
	//template<typename T>
	//void FillImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, std::vector<Eigen::Vector3d>& data,
	//	T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val);
	//
	//template<typename T>
	//void RetrieveImageBlocksRaster(cimg_library::CImg<T>& image, size_t block_size, std::vector<Eigen::Vector3d>& output_data,
	//	T min_img_val, T max_img_val, Eigen::Vector3d min_data_val, Eigen::Vector3d max_data_val);

	void GetWaveletCoefficients(size_t index_offset, std::vector<std::tuple<size_t, size_t, std::unordered_set<size_t>>>& adjacencies,
		std::vector<Eigen::Vector3d>& displacements);

	void InverseWaveletCoefficients(size_t index_offset, std::vector<std::tuple<size_t, size_t, std::unordered_set<size_t>>>& adjacencies,
		std::vector<Eigen::Vector3d>& displacements);
};