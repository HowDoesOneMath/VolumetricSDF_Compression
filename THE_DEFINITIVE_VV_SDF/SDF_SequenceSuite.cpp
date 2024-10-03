#include "SDF_SequenceSuite.h"

void SDF_SequenceSuite::InitializeGrid()
{
	gds.dim_x = width_voxels;
	gds.dim_y = 2 * width_voxels;
	gds.dim_z = width_voxels;

	gds.unit_length = width_meters / (width_voxels - 1);

	gds.center_y = center_meters;


	sdf.InitializeGrid(gds);
}

void SDF_SequenceSuite::LoadSequenceNames()
{
	sf.FindFiles(sequence_folder, mesh_sf);
	sf.FindFiles(sequence_folder, texture_sf);
}

void SDF_SequenceSuite::SaveWholeSequence()
{
	std::vector<size_t> frame_locations;
	frame_locations.resize(sf.files[mesh_sf.key].size());


	buffer.OpenWriteBuffer(save_file);

	//TODO: write GDS data into header! Reserve space for frame nums!

	for (int i = 0; i < sf.files[mesh_sf.key].size(); ++i)
	{
		std::cout << "Saving frame " << i << "..." << std::endl;

		input_mesh.ReadOBJ(sf.files[mesh_sf.key][i]);

		sdf.CastMeshUnsignedDistance(&input_mesh, unsigned_buffer_distance);

		rle.GetRunLength((unsigned char*)sdf.GetQuantizedGridPointer(), sdf.GetQuantizedGridLength());

		buffer.WriteVectorToBuffer(rle.lengths);
		buffer.WriteVectorToBuffer(rle.runs);
	}

	//TODO: Write location of frame nums!

	buffer.CloseWriteBuffer();
}

void SDF_SequenceSuite::SaveSequenceAsPureMeshes()
{
	for (int i = 0; i < sf.files[mesh_sf.key].size(); ++i)
	{
		std::cout << "Saving frame " << i << "..." << std::endl;

		remapped_tex.assign(remapped_texture_dims.x(), remapped_texture_dims.y(), 1, 3, '\0');
		original_tex.assign(sf.files[texture_sf.key][i].c_str());

		input_mesh.ReadOBJ(sf.files[mesh_sf.key][i]);
		mp.CreateUnionFindPartitions(input_mesh);
		mp.NegateInsignificantPartitions(input_mesh, artifact_threshold);
		input_mesh.ClearNegativeTriangles();
		mp.ClearPartitions();

		std::cout << "\tCasting Mesh..." << std::endl;
		sdf.CastMeshUnsignedDistance(&input_mesh, unsigned_buffer_distance);

		std::cout << "\tExtracting Quantized Mesh..." << std::endl;
		auto new_mesh = sdf.ExtractMeshQuantized();// partition_estimate, partition_buffer);

		std::cout << "\tRemapping Texture..." << std::endl;
		tr.RemapWithPartitions(input_mesh, *new_mesh, original_tex, remapped_tex, partition_estimate, partition_buffer, 3);
		
		std::cout << "\tSaving to Disk..." << std::endl;
		new_mesh->WriteOBJ(mesh_file_tag + GetNumberFixedLength(i, number_fixed_length) + mesh_file_ext);
		remapped_tex.save_png((texture_file_tag + GetNumberFixedLength(i, number_fixed_length) + texture_file_ext).c_str());
	}
}

void SDF_SequenceSuite::run(int argc, char** argv)
{
	InitializeGrid();

	LoadSequenceNames();

	//SaveWholeSequence();
	SaveSequenceAsPureMeshes();
}
