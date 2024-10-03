#include "DracoCompressor.h"

DracoCompressor::DracoCompressor(std::vector<std::pair<draco::GeometryAttribute::Type, int>>& quantization_levels,
	int encoding_speed, int decoding_speed)
{
	enc.SetSpeedOptions(encoding_speed, decoding_speed);

	for (size_t i = 0; i < quantization_levels.size(); ++i)
	{
		enc.SetAttributeQuantization(quantization_levels[i].first, quantization_levels[i].second);
	}
}

size_t DracoCompressor::CompressMeshToVector(VV_Mesh &input_mesh, std::vector<char>& output_buffer)
{
	auto draco_mesh = input_mesh.ToDracoMesh();
	enc.EncodeMeshToBuffer(*draco_mesh, &eb);

	size_t buffer_size = eb.size();
	output_buffer.resize(output_buffer.size() + buffer_size);
	memcpy(output_buffer.data(), eb.data(), buffer_size);

	eb.Clear();

	return buffer_size;
}

size_t DracoCompressor::CompressMeshToBuffer(VV_Mesh &input_mesh, VV_SaveFileBuffer& sfb)
{
	auto draco_mesh = input_mesh.ToDracoMesh();
	enc.EncodeMeshToBuffer(*draco_mesh, &eb);

	size_t buffer_size = eb.size();
	sfb.WriteObjectToBuffer(buffer_size);
	sfb.WriteArrayToBuffer(eb.data(), buffer_size);

	eb.Clear();

	return buffer_size;
}

bool DracoCompressor::DecompressMeshFromVector(std::vector<char>& input_buffer, VV_Mesh &output_mesh)
{
	db.Init(input_buffer.data(), input_buffer.size());
	auto status_or_mesh = dec.DecodeMeshFromBuffer(&db);

	if (!status_or_mesh.ok())
	{
		std::cout << "Error reading draco mesh from buffer!" << std::endl;
		return false;
	}

	output_mesh.FromDracoMesh(*(status_or_mesh.value()));

	return true;
}

bool DracoCompressor::DecompressMeshFromBuffer(VV_SaveFileBuffer& sfb, VV_Mesh &output_mesh)
{
	size_t buffer_size;
	sfb.ReadObjectFromBuffer(buffer_size);
	
	char* draco_buffer = new char[buffer_size];
	sfb.ReadArrayFromBuffer(draco_buffer, buffer_size);

	db.Init(draco_buffer, buffer_size);
	auto status_or_mesh = dec.DecodeMeshFromBuffer(&db);

	delete[] draco_buffer;

	if (!status_or_mesh.ok())
	{
		std::cout << "Error reading draco mesh from buffer!" << std::endl;
		return false;
	}

	output_mesh.FromDracoMesh(*(status_or_mesh.value()));

	return true;
}