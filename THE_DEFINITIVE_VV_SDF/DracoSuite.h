#pragma once

#include "TestSuite.h"

#include "SequenceFilePathUniversal.h"

#include "AdditionalUtilities.h"
#include "VV_Mesh.h"

#include <fstream>
#include <string>
#include <vector>

#include <chrono>

#include "DracoCompressor.h"
#include "VV_SaveFileBuffer.h"
#include "SequenceFinder.h"

class DracoSuite : public TestSuite
{
	//std::string mesh_sequence_root = "D:/_VV_DATASETS/_VOLOGRAMS/RAFA/Rafa_Approves_hd_4k";
	std::string mesh_sequence_root = "D:/_VV_DATASETS/Drive_Sequences/FIXED_MTL/FIXED_MTL/AB-2punch";
	//std::string compressed_sequence = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/_RAFA_DRACO/RAFA_DRACO_" + DateTodayManual()  + ".drb";
	std::string compressed_sequence = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_CompressedSequences/_AB-2PUNCH_DRACO/AB-2PUNCH_" + DateTodayManual()  + ".drb";
	//std::string mesh_sequence_output = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ReconstructedSequences/_RECONSTRUCTED_RAFA_DRACO";
	std::string mesh_sequence_output = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_MeshInterop/_ReconstructedSequences/_RECONSTRUCTED_AB-2PUNCH_DRACO";

	//std::string output_mesh_signature = "RAFA_DRACO_RECONSTRUCTED";
	std::string output_mesh_signature = "AB-2PUNCH_DRACO_RECONSTRUCTED";


	VV_Mesh vv_mesh;
	VV_SaveFileBuffer sfb;

	//int position_quant_level = 32;
	//int uv_quant_level = 32;
	//int normal_quant_level = 32;
	int position_quant_level = 11;
	int uv_quant_level = 10;
	int normal_quant_level = 8;
	//int position_quant_level = 11;
	//int uv_quant_level = 10;
	//int normal_quant_level = 8;

	std::vector<std::pair<draco::GeometryAttribute::Type, int>> quantization_options = {
		std::make_pair(draco::GeometryAttribute::Type::POSITION, position_quant_level),
		std::make_pair(draco::GeometryAttribute::Type::TEX_COORD, uv_quant_level),
		std::make_pair(draco::GeometryAttribute::Type::NORMAL, normal_quant_level)
	};

	//int universal_draco_speed = 10;
	int universal_draco_speed = 7;
	//int universal_draco_speed = 0;
	int enc_speed = universal_draco_speed;
	int dec_speed = universal_draco_speed;


	//std::string input_folder = GetDatasetsPath() + "/AB-2punch";
	//std::string sequence_file_identifier = "/AB-2PUNCH";
	
	//std::string input_folder = GetDatasetsPath() + "/AB-dodgeLeft";
	//std::string sequence_file_identifier = "/AB-DODGE";
	
	//std::string input_folder = GetDatasetsPath() + "/AB-death";
	//std::string sequence_file_identifier = "/AB-DEATH";
	
	//std::string input_folder = GetDatasetsPath() + "/Basketball";
	//std::string sequence_file_identifier = "/BASKETBALL";
	
	//std::string input_folder = GetDatasetsPath() + "/RAFA";
	//std::string sequence_file_identifier = "/RAFA";
	
	//std::string input_folder = GetDatasetsPath() + "/LEVI";
	//std::string sequence_file_identifier = "/LEVI";

	std::string input_folder = GetDatasetsPath() + "/SIR_FREDRICK";
	std::string sequence_file_identifier = "/SIR_FREDRICK";


	std::string draco_code = "/_DRACO";


	std::string draco_compression_level = "/COMPRESSION_SPEED_" + std::to_string(universal_draco_speed);
	std::string draco_quant_levels = "/QUANT_" + std::to_string(position_quant_level) + "_" + std::to_string(uv_quant_level) + "_" + std::to_string(normal_quant_level);


	std::string compressed_sequence_folder = GetCompressionPath() + draco_code + draco_compression_level + draco_quant_levels + sequence_file_identifier;
	std::string reconstructed_sequence_folder = GetReconstructionPath() + draco_code + draco_compression_level + draco_quant_levels + sequence_file_identifier;

	std::string compressed_file = compressed_sequence_folder + "/DRACO_COMPRESSED.drc";
	std::string output_mesh_tag = reconstructed_sequence_folder + "/FRAME";
	std::string output_texture_tag = output_mesh_tag;


	std::string time_log_identifier = "/TimeLog.txt";
	std::string time_log_path = compressed_sequence_folder + time_log_identifier;


	DracoCompressor dc = DracoCompressor(quantization_options, enc_speed, dec_speed);

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinderDetails texture_sf = SequenceFinderDetails("Texture", ".jpg");
	SequenceFinder sf;

	bool MassCompress(std::string input_folder, std::string output_file, std::string readfile_tag, int compression_level);

	bool MassDecompress(std::string input_file, std::string output_folder, std::string writefile_tag);

public:
	void run(int argc, char** argv);
};