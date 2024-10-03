#pragma once

#include "TestSuite.h"

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

	int position_quant_level = 11;
	int uv_quant_level = 10;
	int normal_quant_level = 8;

	std::vector<std::pair<draco::GeometryAttribute::Type, int>> quantization_options = {
		std::make_pair(draco::GeometryAttribute::Type::POSITION, position_quant_level),
		std::make_pair(draco::GeometryAttribute::Type::TEX_COORD, uv_quant_level),
		std::make_pair(draco::GeometryAttribute::Type::NORMAL, normal_quant_level)
	};

	int universal_draco_speed = 7;
	int enc_speed = universal_draco_speed;
	int dec_speed = universal_draco_speed;

	DracoCompressor dc = DracoCompressor(quantization_options, enc_speed, dec_speed);

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");
	SequenceFinder sf;

	bool MassCompress(std::string input_folder, std::string output_file, std::string readfile_tag, int compression_level);

	bool MassDecompress(std::string input_file, std::string output_folder, std::string writefile_tag);

public:
	void run(int argc, char** argv);
};