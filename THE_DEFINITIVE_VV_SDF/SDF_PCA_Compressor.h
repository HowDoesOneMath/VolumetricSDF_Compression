#pragma once

#include "VV_Mesh.h"
#include "VV_TSDF.h"
#include "VV_SaveFileBuffer.h"

#include "RunLengthEncoder.h"
#include "LZ_Encoder.h"
#include "PCA_Encoder.h"
#include "SequenceFinder.h"

#include "MeshPartition.h"

#include <chrono>

class SDF_PCA_Compressor
{
	struct PCA_Dimensions
	{
		//Auxiliary grid data
		GridDataStruct gds;

		//Total number of frames in this sequence
		size_t frame_count;

		//The size of a block in the X dimension
		size_t block_size_x;
		//The size of a block in the Y dimension
		size_t block_size_y;
		//The size of a block in the Z dimension
		size_t block_size_z;

		//The amount of blocks along the X dimension per frame
		size_t partitions_x;
		//The amount of blocks along the Y dimension per frame
		size_t partitions_y;
		//The amount of blocks along the Z dimension per frame
		size_t partitions_z;

		//How many blocks in the X dimension compose one input matrix.
		size_t pca_reach_x;
		//How many blocks in the Y dimension compose one input matrix
		size_t pca_reach_y;
		//How many blocks in the Z dimension compose one input matrix
		size_t pca_reach_z;
		//How many blocks in the Time dimension compose one input matrix
		size_t pca_reach_t;

		//Maximum number of energy values to keep from SVD. Initialize to SIZE_MAX to keep all energy values (not here, as it will be overwritten).
		size_t max_allowed_components;

		//Maximum ratio of sum of energy values to keep from SVD. Initialize to > 1.0 to keep all energy values (not here, as it will be overwritten).
		double significant_value_ratio;

		//Variables located below this function are recalculated inside this function to avoid unnnecessary clutter in the save file.
		void CalculateHelperValues();

		//Total amount of block partitions (x * y * z)
		size_t total_partitions;
		
		//Total size of a block (x * y * z)
		size_t total_block_size;

		//Assuming raster order, the distance between blocks in the X dimension in a grid (y * z)
		size_t partition_span_x;
		//Assuming raster order, the distance between blocks in the Y dimension in a grid (z)
		size_t partition_span_y;
		//Assuming raster order, the distance between blocks in the Time dimension in a grid (x * y * z)
		size_t partition_span_t;

		//The distance between blocks in the Time dimension for one input matrix (x * y * z)
		size_t pca_reach_per_frame;
		//The total amount of data contained in a single input matrix (t * x * y * z)
		size_t total_pca_reach;

		//Size of a single grid (x * y * z)
		size_t single_frame_size;

		//Amount of blocks in the X dimension in a grid
		size_t pca_sections_x;
		//Amount of blocks in the Y dimension in a grid
		size_t pca_sections_y;
		//Amount of blocks in the Z dimension in a grid
		size_t pca_sections_z;

		//Assuming raster order, the distance between blocks in the X dimension in an input matrix (y * z)
		size_t pca_span_x;
		//Assuming raster order, the distance between blocks in the Y dimension in an input matrix (z)
		size_t pca_span_y;
		//Assuming raster order, the distance between blocks in the Time dimension in an input matrix (x * y * z)
		size_t pca_span_t;

		//Assuming raster order, the distance between input matrices in the X dimension (y * z)
		size_t pca_reach_span_x;
		//Assuming raster order, the distance between input matrices in the Y dimension (z)
		size_t pca_reach_span_y;
		//Assuming raster order, the distance between input matrices in the Time dimension (x * y * z)
		size_t pca_reach_span_t;

		//Function to save data
		void WriteToBuffer(VV_SaveFileBuffer& buffer);
		//Function to read data
		void ReadFromBuffer(VV_SaveFileBuffer& buffer);

		//Debugging function
		void DebugStats();
	};

	PCA_Dimensions pcad;

	VV_SaveFileBuffer save_buffer;

	VV_Mesh mesh;
	VV_TSDF sdf;
	MeshPartition mp;

	RunLengthEncoder rle_enc;
	LZ_Encoder lz_enc;
	PCA_Encoder pca_enc;

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinder sf;
	
	//int matrix_truncation_maximum = 1000000000;
	//int matrix_truncation_maximum = 255;
	//int matrix_truncation_maximum = 10;
	//int matrix_truncation_maximum = 1;
	unsigned char matrix_truncation_maximum = 255;
	//unsigned char matrix_truncation_maximum = 63;
	//unsigned char matrix_truncation_maximum = 15;
	//unsigned char matrix_truncation_maximum = 3;
	//unsigned char matrix_truncation_maximum = 1;

	size_t GetMultiplexedPartition(size_t x, size_t y, size_t z, size_t t);

	void ExtractSpatialBlockFromPCA(size_t file_location, size_t target_frame, size_t start_x, size_t start_y, size_t start_z);

	void CarryOverRLE_InRange(size_t start_t, size_t end_t, 
		size_t start_x, size_t start_y, size_t start_z, std::vector<size_t>& rle_file_locations);

	size_t SignificantValueCountInPCA(PCA_Dimensions& pca_dim);

	bool ParseFlag(size_t& to_parse, size_t flag);

	std::shared_ptr<VV_Mesh> ExtractSingleMeshFromIntermediary(std::vector<size_t>& file_locations, size_t mesh_index);
	
	size_t MarshallRLE_toPCA(size_t start_t, size_t end_t, size_t start_x, size_t start_y, size_t start_z, 
		std::vector<size_t> &rle_file_locations, std::vector<unsigned char>& block_buffer);

	size_t MarshallLZ_toPCA(size_t start_t, size_t end_t, size_t start_x, size_t start_y, size_t start_z,
		std::vector<size_t>& lz_file_locations, std::vector<unsigned char>& block_buffer);

	std::shared_ptr<VV_Mesh> ExtractSingleMesh(std::vector<size_t>& block_locations, std::vector<size_t>& sign_locations, size_t mesh_index_in_SVD);

	bool FillPCA_Struct(GridDataStruct& gds, Eigen::Vector3i block_size, Eigen::Vector4i pca_reach, 
		double significant_value_ratio, size_t max_allowed_components);

	void PrintSingularValues(Eigen::VectorXd &values, size_t value_limit);

	void PrintSingularValuesAndCoords(size_t x, size_t y, size_t z, size_t t, Eigen::VectorXd& values, size_t value_limit);

	template<typename T>
	void PrintMatrix(Eigen::MatrixX<T> &to_print, size_t r_start, size_t c_start, size_t r_end, size_t c_end, bool row_major = false);

public:
	bool PreInitialization(GridDataStruct& gds, Eigen::Vector3i block_size, Eigen::Vector4i pca_reach,
		double significant_value_ratio, size_t max_allowed_components);

	bool SaveIntermediaryFile(std::string input_folder, std::string intermediary_file_name, 
		double shell_buffer, double mesh_maximum_artifact_size);

	bool SavePCA_File(std::string intermediary_file_name, std::string pca_file_name);

	bool ReconstructMeshesFromPCA(std::string pca_file_name, std::string output_folder, std::string output_file_tag, int digit_count);

	bool ReconstructMeshesFromIntermediary(std::string intermediary_file_name, std::string output_folder, std::string output_file_tag, int digit_count);

	/// <summary>
	/// This primarily exists because of a non-trivial bug that occured as I was saving PCA values, regarding Eigen::BDCSVD arbitrarily crashing for some matrices.
	/// Should not be used anymore, other than for debugging purposes.
	/// </summary>
	/// <param name="gds"></param>
	/// <param name="input_folder"></param>
	/// <param name="target_location"></param>
	/// <param name="block_size"></param>
	/// <param name="shell_buffer"></param>
	/// <param name="pca_reach"></param>
	/// <param name="significant_value_threshold"></param>
	/// <param name="max_allowed_components"></param>
	/// <param name="mesh_maximum_artifact_size"></param>
	/// <returns></returns>
	bool TestSpecificSection(GridDataStruct& gds, std::string input_folder, Eigen::Vector4i target_location, Eigen::Vector3i block_size, 
		double shell_buffer, Eigen::Vector4i pca_reach, double significant_value_threshold, size_t max_allowed_components, double mesh_maximum_artifact_size);
};

template<typename T>
inline void SDF_PCA_Compressor::PrintMatrix(Eigen::MatrixX<T>& to_print, size_t r_start, size_t c_start, size_t r_end, size_t c_end, bool row_major)
{
	if (row_major)
	{
		for (size_t r = r_start; r < r_end; ++r)
		{
			for (size_t c = c_start; c < c_end; ++c)
			{
				std::cout << to_print(r, c) << "\t";
			}

			std::cout << "\n";
		}
	}
	else
	{
		for (size_t c = c_start; c < c_end; ++c)
		{
			for (size_t r = r_start; r < r_end; ++r)
			{
				std::cout << to_print(r, c) << "\t";
			}

			std::cout << "\n";
		}
	}

	std::cout << std::endl;
}
