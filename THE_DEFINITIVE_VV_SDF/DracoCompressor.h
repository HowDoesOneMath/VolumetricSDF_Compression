#pragma once

#include "AdditionalUtilities.h"
#include "VV_Mesh.h"

#include "VV_SaveFileBuffer.h"

#include "SequenceFinder.h"

#include <fstream>
#include <string>
#include <vector>

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

class DracoCompressor
{
	VV_Mesh vv_mesh;

	draco::ObjDecoder obj_decoder;
	draco::ObjEncoder obj_encoder;

	draco::EncoderBuffer eb;
	draco::DecoderBuffer db;

	draco::Decoder dec;
	draco::Encoder enc;

public:
	DracoCompressor(std::vector<std::pair<draco::GeometryAttribute::Type, int>>& quantization_levels, 
		int encoding_speed = 7, int decoding_speed = 7);

	size_t CompressMeshToVector(VV_Mesh& input_mesh, std::vector<char> &output_buffer);
	size_t CompressMeshToBuffer(VV_Mesh& input_mesh, VV_SaveFileBuffer &sfb);

	bool DecompressMeshFromVector(std::vector<char>& input_buffer, VV_Mesh& output_mesh);
	bool DecompressMeshFromBuffer(VV_SaveFileBuffer& sfb, VV_Mesh& output_mesh);
};