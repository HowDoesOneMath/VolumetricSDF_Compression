#pragma once

#include "AdditionalUtilities.h"
#include "VV_Mesh.h"

#include "TimeLogger.h"

#include "VV_SaveFileBuffer.h"

#include "SequenceFinder.h"

#include <fstream>
#include <string>
#include <vector>

#include <CImg.h>

#include <draco/compression/encode.h>
#include <draco/compression/decode.h>

#include <draco/io/file_reader_factory.h>
#include <draco/io/file_writer_factory.h>

#include <draco/io/stdio_file_reader.h>
#include <draco/io/stdio_file_writer.h>

#include <draco/mesh/mesh.h>
#include <draco/io/mesh_io.h>
#include <draco/io/obj_decoder.h>
#include <draco/io/obj_encoder.h>

//#define DRACO_TIME_LOGGING 0
#define DRACO_TIME_LOGGING 1

class DracoCompressor
{
	VV_SaveFileBuffer sfb;
	SequenceFinder sf;

	VV_Mesh vv_mesh;

	draco::ObjDecoder obj_decoder;
	draco::ObjEncoder obj_encoder;

	draco::EncoderBuffer eb;
	draco::DecoderBuffer db;

	draco::Decoder dec;
	draco::Encoder enc;

#if DRACO_TIME_LOGGING
	TimeLogger tl;

	std::string total_time_logger_name				= "TOTAL_TIME_LOG";

	std::string draco_time_logger_name				= "DRACO_TIME_LOG";
	std::string draco_mesh_conversion_logger_name	= "MESH_CONVERSION_TIME_LOG";
	std::string header_time_logger_name				= "HEADER_TIME_LOG";
	std::string obj_reading_logger_name				= "OBJ_READING_TIME_LOG";
	std::string obj_writing_logger_name				= "OBJ_WRITING_TIME_LOG";
	std::string texture_copying_logger_name			= "TEX_COPY_TIME_LOG";
	std::string file_writing_logger_name			= "FILE_WRITING_TIME_LOG";
	std::string file_reading_logger_name			= "FILE_READING_TIME_LOG";
#endif

public:

#if DRACO_TIME_LOGGING
	void CreateTimeLog(std::string filename);

	void CloseTimeLog();
#endif

	DracoCompressor(std::vector<std::pair<draco::GeometryAttribute::Type, int>>& quantization_levels, 
		int encoding_speed = 7, int decoding_speed = 7);

	size_t CompressMeshToVector(VV_Mesh& input_mesh, std::vector<char> &output_buffer);
	size_t CompressMeshToBuffer(VV_Mesh& input_mesh, VV_SaveFileBuffer &sfb);

	bool DecompressMeshFromVector(std::vector<char>& input_buffer, VV_Mesh& output_mesh);
	bool DecompressMeshFromBuffer(VV_SaveFileBuffer& sfb, VV_Mesh& output_mesh);

	bool CompressSequence(std::string root_folder, std::string compressed_file_name, 
		SequenceFinderDetails mesh_sf, SequenceFinderDetails texture_sf, std::string texture_name, int jpg_quality = 100);
	bool DecompressSequence(std::string compressed_file_name, std::string mesh_name);
};