#pragma once

#include "SequenceFinder.h"
#include "VV_SaveFileBuffer.h"

#include "MeshPartition.h"
#include "VV_TSDF.h"
#include "PCA_Encoder.h"

#include "LZ_Encoder.h"
#include "SDF_RotationCaliper.h"

#include "TextureRemapper.h"

#include "TimeLogger.h"

#include <string>

//#define ENCODE_SIGN_DATA 1;
#define ENCODE_SIGN_DATA 0;

//#define TSVD_TIME_LOGGING 0
#define TSVD_TIME_LOGGING 1


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

#if TSVD_TIME_LOGGING
	TimeLogger tl;

	std::string total_time_logger_name = "TOTAL_TIME_LOG";
	std::string dummy_time_logger_name = "DUMMY_TIME_LOG"; //Tabbed

	std::string header_intermediary_time_logger_name		= "HEADER_INTERMEDIARY_TIME_LOG";
	std::string mesh_reading_time_logger_name				= "MESH_READING_TIME_LOG";
	std::string mesh_cleaning_time_logger_name				= "MESH_CLEANING_TIME_LOG";
	std::string mesh_to_SDF_time_logger_name				= "MESH_TO_SDF_TIME_LOG";
	std::string block_extraction_logger_name				= "BLOCK_EXTRACT_TIME_LOG";
	std::string lz_encoding_logger_name						= "LZ_ENCODING_TIME_LOG";
	std::string sign_encoding_logger_name					= "SIGN_ENCODING_TIME_LOG";

	std::string header_TSVD_time_logger_name				= "HEADER_TSVD_TIME_LOG";
	std::string sign_copy_time_logger_name					= "SIGN_COPY_TIME_LOG";
	std::string batch_TSVD_time_logger_name					= "BATCH_TSVD_TIME_LOG";
	std::string block_loading_time_logger_name				= "BLOCK_LOADING_TIME_LOG"; //Tabbed
	std::string SVD_time_logger_name						= "SVD_TIME_LOG"; //Tabbed
	std::string matrix_writing_time_logger_name				= "MAT_WRITE_TIME_LOG"; // Tabbed

	std::string header_textures_time_logger_name			= "HEADER_TEXTURE_TIME_LOG";
	std::string important_blocks_logger_name				= "PATCH_BLOCKS_TIME_LOG"; //Tabbed
	std::string patch_creation_texturing_logger_name		= "UV_PATCH_TEX_TIME_LOG"; //Tabbed
	std::string texture_batch_time_logger_name				= "TEXTURE_BATCH_TIME_LOG";
	std::string mesh_uv_creating_logger_name				= "MESH_UV_TIME_LOG";
	std::string texture_remapping_logger_name				= "TEXTURE_REMAP_TIME_LOG";
	std::string texture_saving_logger_name					= "TEXTURE_SAVE_TIME_LOG";

	std::string header_reconstruction_time_logger_name		= "HEADER_RECON_TIME_LOG";
	std::string mesh_reconstruction_logger_name				= "MESH_RECON_TIME_LOG";
	std::string patch_creation_reconstruction_logger_name	= "UV_PATCH_RECON_TIME_LOG";
	std::string mesh_uv_reconstruction_logger_name			= "MESH_UV_RECON_TIME_LOG";
	std::string mesh_obj_writing_logger_name				= "OBJ_WRITING_TIME_LOG";

#endif

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
		std::vector<size_t>& block_locations, size_t start_ind, size_t end_ind);

	void CreateUVs(VV_Mesh& target_mesh, std::vector<std::pair<size_t, Eigen::Vector2i>>& patch_locations, Eigen::Vector2d patch_spacing);

	void DebugLocationsVector(std::vector<size_t>& to_debug, std::string message, size_t frequency, size_t offset);

public:

#if TSVD_TIME_LOGGING
	void SetTimeLogFile(std::string time_log_name);

	void CloseTimeLogFile();
#endif

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