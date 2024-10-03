#pragma once

#include "SequenceFinder.h"
#include "VV_SaveFileBuffer.h"

#include "MeshPartition.h"
#include "VV_TSDF.h"
#include "PCA_Encoder.h"

#include "LZ_Encoder.h"
#include "SDF_RotationCaliper.h"

#include "TextureRemapper.h"

#include <string>

//#define ENCODE_SIGN_DATA 1;
#define ENCODE_SIGN_DATA 0;

class VV_SVD_TemporalCompressor
{
public:
	struct SVD_AdditionalData
	{
		//Additional grid data
		GridDataStruct gds;

		//Size of blocks for input matrix
		Eigen::Vector3i block_size;

		//Amount of frames per input matrix
		size_t frames_per_input_matrix;

		//Total amount of frames in sequence
		size_t total_frames;

		//The amount of padding to add to the uv coord patches, helps prevent overlapping pixels. Measured as a 0-1 percentage of patch space.
		Eigen::Vector2d patch_padding;

		//How similar normals can be before they are designated as a new patch
		double minimum_normal_similarity;

		//The amount of padding per uv island within patches. This measurement is related to the mesh size, but ultimately inexact as patches are scaled after padding.
		double island_padding;

		void RecalculateStats();

		//The total elements per block
		size_t total_block_size;

		//The amount of blocks in x, y, z dimensions
		Eigen::Vector3i partitions;

		//The total amount of partitions
		size_t total_partitions;

		//Assuming raster order, the distance between blocks in the X dimension in a grid (y * z)
		size_t partition_span_x;
		//Assuming raster order, the distance between blocks in the Y dimension in a grid (z)
		size_t partition_span_y;
		//Assuming raster order, the distance between blocks in the Time dimension in a grid (x * y * z)
		size_t partition_span_t;

		//The scaling of uv patches, determined by the padding
		Eigen::Vector2d patch_scale;

		void WriteToBuffer(VV_SaveFileBuffer &buff);
		void ReadFromBuffer(VV_SaveFileBuffer &buff);
	};

private:
	std::vector<unsigned char> block_buffer;

	VV_SaveFileBuffer sfb;

	VV_Mesh mesh;
	cimg_library::CImg<unsigned char> tex;
	MeshPartition mp;

	VV_TSDF sdf;
	PCA_Encoder pca_enc;
	LZ_Encoder lz_enc;

	TextureRemapper tr;
	SDF_RotationCaliper rc;

	SVD_AdditionalData sad;
	SequenceFinder sf;

	unsigned char matrix_truncation_maximum = 255;

	const size_t important_geometry_flag = (((size_t)1) << 63);
	const size_t important_geometry_mask = ~important_geometry_flag;

	size_t GetSignificantValueCount(double significant_value_ratio, size_t max_allowed_components);

	void CopySignsOver();

	void UseSigns(std::vector<unsigned char> sign_data);

	void ConstructBlock(std::vector<size_t>& block_locations, std::vector<size_t>& svd_locations, 
		size_t svd_loc, size_t start_ind, size_t end_ind, double significant_value_ratio, size_t max_allowed_components);

	std::shared_ptr<VV_Mesh> ExtractSingleMesh(std::vector<size_t>& block_locations, std::vector<size_t>& sign_locations, size_t mesh_index_in_SVD);

	void EncodeImportantBlocks(size_t start_t, size_t end_t, std::vector<size_t>& block_locations, std::vector<size_t>& sign_locations);

	std::shared_ptr<std::pair<std::vector<std::pair<size_t, Eigen::Vector2i>>, Eigen::Vector2i>> GetPatchInfo(
		std::vector<size_t>& block_locations);

	void CreateUVs(VV_Mesh& target_mesh, std::vector<std::pair<size_t, Eigen::Vector2i>>& patch_locations, Eigen::Vector2d patch_spacing);

	void DebugLocationsVector(std::vector<size_t>& to_debug, std::string message, size_t frequency, size_t offset);

public:
	bool Initialize(GridDataStruct& gds, Eigen::Vector3i block_size, size_t frames_per_batch,
		double patch_padding, double island_padding, double minimum_normal_similarity);

	bool SaveIntermediaryFile(std::string root_folder, std::string intermediary_file_name, SequenceFinderDetails mesh_sf, 
		double shell_buffer, double mesh_maximum_artifact_size);

	bool SaveFinalFile(std::string intermediary_file_name, std::string final_file_name, 
		double significant_value_ratio, size_t max_allowed_components);

	bool AugmentFinalFileWithTextureData(std::string root_folder, SequenceFinderDetails mesh_sf, SequenceFinderDetails texture_sf,
		std::string final_file_name, std::string texture_tag, int digit_count, Eigen::Vector2i texture_dims,
		double uv_epsilon, int kernel_size = 5, double kernel_scale = 2.0, int jpg_quality = 100);

	/// <summary>
	/// Reconstruct all meshes from a file.
	/// !WARNING! - this makes no attempt to discern if a file has sign data or not, only relying on the #define in the header.
	/// Do not try to make it read sign data if there is none, it will crash!
	/// </summary>
	/// <param name="final_file_name">File to read from</param>
	/// <param name="output_mesh_tag">Tag that is appended to the front of all meshes, must include a path as well. Do not include a file extension</param>
	/// <param name="digit_count">How many digits to include in the trailing number for output files (e.g. 4 digits: 'Mesh_0001.obj')</param>
	/// <returns></returns>
	bool ReconstructMeshes(std::string final_file_name, std::string output_mesh_tag, int digit_count);
};