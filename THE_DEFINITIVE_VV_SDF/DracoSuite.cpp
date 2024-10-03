#include "DracoSuite.h"

//void DracoSuite::InitializeDracoEncoderDecoder()
//{
//	draco::FileWriterFactory::RegisterWriter(draco::StdioFileWriter::Open);
//	draco::FileReaderFactory::RegisterReader(draco::StdioFileReader::Open);
//}

bool DracoSuite::MassCompress(std::string input_folder, std::string output_file, std::string readfile_tag, int compression_level)
{
	mesh_sf.file_tag = readfile_tag;
	if (!sf.FindFiles(input_folder, mesh_sf))
	{
		return false;
	}

	std::vector<size_t> file_starting_locations;
	size_t file_header_starts;

	size_t file_count = sf.files[mesh_sf.key].size();

	if (!sfb.OpenWriteBuffer(output_file))
	{
		std::cout << "Couldn't open writer!" << std::endl;
		return false;
	}

	sfb.WriteObjectToBuffer(file_count);
	file_header_starts = sfb.GetWriterLocation();

	file_starting_locations.resize(file_count);
	sfb.WriteArrayToBuffer(file_starting_locations.data(), file_starting_locations.size());

	for (size_t i = 0; i < file_count; ++i)
	{
		std::cout << sf.files[mesh_sf.key][i] << std::endl;
		std::cout << "ENC: " << i << std::endl;

		vv_mesh.ReadOBJ(sf.files[mesh_sf.key][i]);
		file_starting_locations[i] = sfb.GetWriterLocation();

		dc.CompressMeshToBuffer(vv_mesh, sfb);
	}

	sfb.SetWriterLocation(file_header_starts);
	sfb.WriteArrayToBuffer(file_starting_locations.data(), file_starting_locations.size());

	sfb.CloseWriteBuffer();

	return true;
}

bool DracoSuite::MassDecompress(std::string input_file, std::string output_folder, std::string writefile_tag)
{
	if (!sfb.OpenReadBuffer(input_file))
	{
		std::cout << "Couldn't open reader!" << std::endl;
		return false;
	}

	std::vector<size_t> file_starts;
	sfb.ReadVectorFromBuffer(file_starts);

	std::string save_name; 

	double total_decompression_time = 0;
	double decompression_time;

	for (size_t i = 0; i < file_starts.size(); ++i)
	{
		save_name = output_folder + "/" + writefile_tag + "_" + GetNumberFixedLength(i, 8) + ".obj";
		std::cout << "DEC: " << i << " --> " << save_name << std::endl;

		auto time_begin = std::chrono::high_resolution_clock::now();

		if (!dc.DecompressMeshFromBuffer(sfb, vv_mesh))
		{
			std::cout << "ERROR: Couldn't get mesh!" << std::endl;
			return false;
		}

		auto time_delta = std::chrono::high_resolution_clock::now() - time_begin;

		decompression_time = time_delta.count() * 0.000000001;
		std::cout << "Elapsed (decompress): " << decompression_time << "s" << std::endl;
		total_decompression_time += decompression_time;

		vv_mesh.WriteOBJ(save_name);
	}

	std::cout << "Total decompression time: " << total_decompression_time << std::endl;

	return true;
}

void DracoSuite::run(int argc, char** argv)
{
	if (!MassCompress(mesh_sequence_root, compressed_sequence, "", 7))
	{
		return;
	}
	
	vv_mesh.Clear();

	if (!MassDecompress(compressed_sequence, mesh_sequence_output, output_mesh_signature))
	{
		return;
	}
}
