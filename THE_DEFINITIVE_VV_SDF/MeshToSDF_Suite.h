#pragma once

#include "TestSuite.h"

#include <string>

#include "SequenceFilePathUniversal.h"
#include "SequenceFinder.h"

#include "VV_Mesh.h"
#include "VV_TSDF.h"

#include "MeshPartition.h"
#include "AdditionalUtilities.h"

class MeshToSDF_Suite : public TestSuite
{
	VV_Mesh mesh;
	VV_TSDF sdf;

	MeshPartition mp;

	SequenceFinder sf;

	SequenceFinderDetails mesh_sf = SequenceFinderDetails("Mesh", ".obj");

	const int digit_count = 6;

	//const size_t grid_width_voxels = 512;
	//const size_t grid_width_voxels = 256;
	const size_t grid_width_voxels = 128;
	//const size_t grid_width_voxels = 64;
	//const double grid_width_meters = 1.6;
	const double grid_width_meters = 1.1;
	//const Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
	const Eigen::Vector3d center = Eigen::Vector3d(0, 1, 0);

	const double shell_size = 0.04;
	const double mesh_maximum_artifact_size = 0.05;

	GridDataStruct gds;


	//std::string input_folder = GetDatasetsPath() + "/AB-2punch";
	std::string input_folder = GetDatasetsPath() + "/SIR_FREDRICK";

	//std::string sequence_file_identifier = "/AB-2PUNCH";
	std::string sequence_file_identifier = "/SIR_FREDRICK";


	std::string output_folder = GetSDF_MeshDatasetsPath() + sequence_file_identifier;

	std::string output_mesh_tag = output_folder + "/FRAME";

	void ConvertSequence();

public:

	void run(int argc, char** argv);
};