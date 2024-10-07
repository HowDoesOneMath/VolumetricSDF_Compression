#include "DracoCompressor.h"

void DracoCompressor::CreateTimeLog(std::string filename)
{
	tl.StartLogging(filename);

	tl.CreateNewLogger(total_time_logger_name, new TimeLogger::IndependentLogger(""));

	tl.CreateNewLogger(draco_time_logger_name, new TimeLogger::IndependentLogger(""));
	tl.CreateNewLogger(draco_mesh_conversion_logger_name, new TimeLogger::IndependentLogger(""));
	tl.CreateNewLogger(header_time_logger_name, new TimeLogger::IndependentLogger(""));
	tl.CreateNewLogger(obj_reading_logger_name, new TimeLogger::IndependentLogger(""));
	tl.CreateNewLogger(obj_writing_logger_name, new TimeLogger::IndependentLogger(""));
	tl.CreateNewLogger(texture_copying_logger_name, new TimeLogger::IndependentLogger(""));
	tl.CreateNewLogger(file_writing_logger_name, new TimeLogger::IndependentLogger(""));
	tl.CreateNewLogger(file_reading_logger_name, new TimeLogger::IndependentLogger(""));

}

void DracoCompressor::CloseTimeLog()
{
	tl.CloseLoggers();
}

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
#if DRACO_TIME_LOGGING
	tl.GetLogger(draco_mesh_conversion_logger_name)->StartTimer();
#endif

	auto draco_mesh = input_mesh.ToDracoMesh();

#if DRACO_TIME_LOGGING
	tl.GetLogger(draco_mesh_conversion_logger_name)->MarkTime();
	tl.GetLogger(draco_time_logger_name)->StartTimer();
#endif

	enc.EncodeMeshToBuffer(*draco_mesh, &eb);

#if DRACO_TIME_LOGGING
	tl.GetLogger(draco_time_logger_name)->MarkTime();
	tl.GetLogger(file_writing_logger_name)->StartTimer();
#endif

	size_t buffer_size = eb.size();
	sfb.WriteObjectToBuffer(buffer_size);
	sfb.WriteArrayToBuffer(eb.data(), buffer_size);

#if DRACO_TIME_LOGGING
	tl.GetLogger(file_writing_logger_name)->MarkTime();
#endif

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
#if DRACO_TIME_LOGGING
	tl.GetLogger(file_reading_logger_name)->StartTimer();
#endif

	size_t buffer_size;
	sfb.ReadObjectFromBuffer(buffer_size);
	
	char* draco_buffer = new char[buffer_size];
	sfb.ReadArrayFromBuffer(draco_buffer, buffer_size);

#if DRACO_TIME_LOGGING
	tl.GetLogger(file_reading_logger_name)->MarkTime();
	tl.GetLogger(draco_time_logger_name)->StartTimer();
#endif

	db.Init(draco_buffer, buffer_size);
	auto status_or_mesh = dec.DecodeMeshFromBuffer(&db);

#if DRACO_TIME_LOGGING
	tl.GetLogger(draco_time_logger_name)->MarkTime();
#endif

	delete[] draco_buffer;

	if (!status_or_mesh.ok())
	{
		std::cout << "Error reading draco mesh from buffer!" << std::endl;
		return false;
	}

#if DRACO_TIME_LOGGING
	tl.GetLogger(draco_mesh_conversion_logger_name)->StartTimer();
#endif

	output_mesh.FromDracoMesh(*(status_or_mesh.value()));

#if DRACO_TIME_LOGGING
	tl.GetLogger(draco_mesh_conversion_logger_name)->MarkTime();
#endif

	return true;
}

bool DracoCompressor::CompressSequence(std::string root_folder, std::string compressed_file_name, 
	SequenceFinderDetails mesh_sf, SequenceFinderDetails texture_sf, std::string texture_name, int jpg_quality)
{
#if DRACO_TIME_LOGGING
	tl.GetLogger(total_time_logger_name)->ResetTotalTime();

	tl.GetLogger(draco_mesh_conversion_logger_name)->ResetTotalTime();
	tl.GetLogger(draco_time_logger_name)->ResetTotalTime();
	tl.GetLogger(header_time_logger_name)->ResetTotalTime();
	tl.GetLogger(obj_reading_logger_name)->ResetTotalTime();
	tl.GetLogger(file_writing_logger_name)->ResetTotalTime();
#endif

	std::vector<SequenceFinderDetails> all_details;
	all_details.push_back(mesh_sf);
	all_details.push_back(texture_sf);

	if (!sf.FindFiles(root_folder, all_details))
	{
		return false;
	}

	std::vector<size_t> file_starting_locations;
	size_t file_header_starts;

	size_t file_count = sf.files[mesh_sf.key].size();

	if (!sfb.OpenWriteBuffer(compressed_file_name))
	{
		std::cout << "Couldn't open writer!" << std::endl;
		return false;
	}

#if DRACO_TIME_LOGGING
	tl.GetLogger(header_time_logger_name)->StartTimer();
#endif

	sfb.WriteObjectToBuffer(file_count);
	file_header_starts = sfb.GetWriterLocation();

	file_starting_locations.resize(file_count);
	sfb.WriteArrayToBuffer(file_starting_locations.data(), file_starting_locations.size());

#if DRACO_TIME_LOGGING
	tl.GetLogger(header_time_logger_name)->MarkTime();
#endif

	cimg_library::CImg<unsigned char> to_copy;

	for (size_t i = 0; i < file_count; ++i)
	{
		std::cout << "ENC: " << i << sf.files[mesh_sf.key][i] << std::endl;

#if DRACO_TIME_LOGGING
		tl.GetLogger(total_time_logger_name)->StartTimer();
		tl.GetLogger(obj_reading_logger_name)->StartTimer();
#endif

		vv_mesh.ReadOBJ(sf.files[mesh_sf.key][i]);

#if DRACO_TIME_LOGGING
		tl.GetLogger(obj_reading_logger_name)->MarkTime();
#endif

		file_starting_locations[i] = sfb.GetWriterLocation();

		CompressMeshToBuffer(vv_mesh, sfb);

#if DRACO_TIME_LOGGING
		tl.GetLogger(texture_copying_logger_name)->StartTimer();
#endif

		to_copy.assign(sf.files[texture_sf.key][i].c_str());
		std::string new_tex_name = texture_name + "_" + GetNumberFixedLength(i, 6) + ".jpg";
		to_copy.save_jpeg(texture_name.c_str(), jpg_quality);

#if DRACO_TIME_LOGGING
		tl.GetLogger(texture_copying_logger_name)->MarkTime();
		tl.GetLogger(total_time_logger_name)->MarkTime();
		std::cout << "Time for this frame: " << (tl.GetLogger(total_time_logger_name)->GetTime() * 0.000000001) << " seconds." << std::endl;
#endif
	}

#if DRACO_TIME_LOGGING
	tl.GetLogger(header_time_logger_name)->StartTimer();
#endif

	sfb.SetWriterLocation(file_header_starts);
	sfb.WriteArrayToBuffer(file_starting_locations.data(), file_starting_locations.size());

#if DRACO_TIME_LOGGING
	tl.GetLogger(header_time_logger_name)->MarkTime();
#endif

	sfb.CloseWriteBuffer();


#if DRACO_TIME_LOGGING
	tl.PrintTotalAndAverageAndGreatestTime(total_time_logger_name, "Total time", "Average time", "Greatest time", " of Compression: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(header_time_logger_name, "Total time", "Average time", "Greatest time", " of Header Writing: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(obj_reading_logger_name, "Total time", "Average time", "Greatest time", " of OBJ Reading: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(draco_mesh_conversion_logger_name, "Total time", "Average time", "Greatest time", " of Draco Mesh Conversion: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(draco_time_logger_name, "Total time", "Average time", "Greatest time", " of Draco Compression Algorithm: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(file_writing_logger_name, "Total time", "Average time", "Greatest time", " of File Writing: ");
	tl.PrintEmptyLine(); 
	tl.PrintTotalAndAverageAndGreatestTime(texture_copying_logger_name, "Total time", "Average time", "Greatest time", " of Texture Copying: ");
	tl.PrintEmptyLine();
	tl.PrintEmptyLine();
	tl.PrintSolidLine(40, '=');
#endif


	return true;
}

bool DracoCompressor::DecompressSequence(std::string compressed_file_name, std::string mesh_name)
{
#if DRACO_TIME_LOGGING
	tl.GetLogger(total_time_logger_name)->ResetTotalTime();

	tl.GetLogger(draco_mesh_conversion_logger_name)->ResetTotalTime();
	tl.GetLogger(file_reading_logger_name)->ResetTotalTime();
	tl.GetLogger(draco_time_logger_name)->ResetTotalTime();
	tl.GetLogger(obj_writing_logger_name)->ResetTotalTime();
#endif

	if (!sfb.OpenReadBuffer(compressed_file_name))
	{
		std::cout << "Couldn't open reader!" << std::endl;
		return false;
	}

	std::vector<size_t> file_starts;
	sfb.ReadVectorFromBuffer(file_starts);

	std::string save_name;


	for (size_t i = 0; i < file_starts.size(); ++i)
	{
#if DRACO_TIME_LOGGING
		tl.GetLogger(total_time_logger_name)->StartTimer();
#endif

		save_name = mesh_name + "_" + GetNumberFixedLength(i, 6) + ".obj";
		std::cout << "DEC: " << i << " --> " << save_name << std::endl;

		if (!DecompressMeshFromBuffer(sfb, vv_mesh))
		{
			std::cout << "ERROR: Couldn't get mesh!" << std::endl;
			return false;
		}

#if DRACO_TIME_LOGGING
		tl.GetLogger(obj_writing_logger_name)->StartTimer();
#endif

		vv_mesh.WriteOBJ(save_name);

#if DRACO_TIME_LOGGING
		tl.GetLogger(obj_writing_logger_name)->MarkTime();
		tl.GetLogger(total_time_logger_name)->MarkTime();
		std::cout << "Time for this frame: " << (tl.GetLogger(total_time_logger_name)->GetTime() * 0.000000001) << " seconds." << std::endl;
#endif
	}


#if DRACO_TIME_LOGGING
	tl.PrintTotalAndAverageAndGreatestTime(total_time_logger_name, "Total time", "Average time", "Greatest time", " of Decompression: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(file_reading_logger_name, "Total time", "Average time", "Greatest time", " of File Reading: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(draco_time_logger_name, "Total time", "Average time", "Greatest time", " of Draco Decompression Algorithm: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(draco_mesh_conversion_logger_name, "Total time", "Average time", "Greatest time", " of Draco Mesh Conversion: ");
	tl.PrintEmptyLine();
	tl.PrintTotalAndAverageAndGreatestTime(obj_writing_logger_name, "Total time", "Average time", "Greatest time", " of OBJ Writing: ");
	tl.PrintEmptyLine();
	tl.PrintEmptyLine();
	tl.PrintSolidLine(40, '=');
#endif


	return true;
}
