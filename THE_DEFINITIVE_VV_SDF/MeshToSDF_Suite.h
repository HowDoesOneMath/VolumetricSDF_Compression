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


	const double mesh_maximum_artifact_size = 0.05;

	GridDataStruct gds;

	//std::string sequence_file_identifier = "/NON_VV_DATASET";
	//std::string input_folder = GetDatasetsPath() + "/_NON_VV_DATASET";
	//std::string sequence_file_identifier = "/AB-2PUNCH";
	//std::string input_folder = GetDatasetsPath() + "/AB-2punch";
	//std::string sequence_file_identifier = "/AB-DODGE";
	//std::string input_folder = GetDatasetsPath() + "/AB-dodgeLeft";
	//std::string sequence_file_identifier = "/AB-DEATH";
	//std::string input_folder = GetDatasetsPath() + "/AB-death";

	//--------------------------------------------------------------------------------------------------------------

	std::string sequence_file_identifier = "/BASKETBALL";
	std::string input_folder = GetDatasetsPath() + "/Basketball";
	 
	const size_t tex_size = 2048;
	 
	Eigen::Vector3d center = Eigen::Vector3d(270, 460, 220);
	const double grid_width_meters = 2100;
	
	//const size_t grid_width_voxels = 128;
	//double shell_size = 56;
	const size_t grid_width_voxels = 256;
	double shell_size = 56; 

	//--------------------------------------------------------------------------------------------------------------

	//std::string sequence_file_identifier = "/RAFA";
	//std::string input_folder = GetDatasetsPath() + "/RAFA";
	// 
	//const size_t tex_size = 4096;
	// 
	//Eigen::Vector3d center = Eigen::Vector3d(0.2, 1, 0);
	//const double grid_width_meters = 1.6;
	//
	//const size_t grid_width_voxels = 64;
	//double shell_size = 0.022;
	////const size_t grid_width_voxels = 128;
	////double shell_size = 0.011;
	////const size_t grid_width_voxels = 256;
	////double shell_size = 0.006; 

	//--------------------------------------------------------------------------------------------------------------

	//std::string sequence_file_identifier = "/LEVI";
	//std::string input_folder = GetDatasetsPath() + "/LEVI";
	// 
	//const size_t tex_size = 4096;
	// 
	//Eigen::Vector3d center = Eigen::Vector3d(0, 1, 0);
	//const double grid_width_meters = 1.8;
	//
	//const size_t grid_width_voxels = 64;
	//double shell_size = 0.025;
	////const size_t grid_width_voxels = 128;
	////double shell_size = 0.013;
	////const size_t grid_width_voxels = 256;
	////double shell_size = 0.007; 

	//--------------------------------------------------------------------------------------------------------------

	//std::string sequence_file_identifier = "/SIR_FREDRICK";
	//std::string input_folder = GetDatasetsPath() + "/SIR_FREDRICK";
	//
	//const size_t tex_size = 4096;
	//
	//Eigen::Vector3d center = Eigen::Vector3d(0, 1, 0);
	//const double grid_width_meters = 1.2;
	//
	//const size_t grid_width_voxels = 64;
	//double shell_size = 0.02;
	////const size_t grid_width_voxels = 128;
	////double shell_size = 0.01;
	////const size_t grid_width_voxels = 256;
	////double shell_size = 0.009; 

	//--------------------------------------------------------------------------------------------------------------


	//SIR FREDRICK STATS: (1.2 meters, center 0, 1, 0): 64 - 0.02, 128 - 0.01, 256 - 0.009
	//	BB: (-0.56531 -0.0376291   -0.31541) - (0.506176  1.89792 0.564333)
	//LEVI STATS: (1.8 meters, center 0, 1, 0): 64 - 0.025, 128 - 0.013, 256 - 0.007
	//	BB: (-0.780687 -0.0424938  -0.594318) - (0.857238  1.90897 0.687259)
	//RAFA STATS: (1.6 meters, center 0.2, 1, 0): 64 - 0.022, 128 - 0.011, 256 - 0.006
	//	BB: (-0.438501 -0.0130447  -0.323075) - (0.876967  1.98185 0.516095)
	//BASKETBALL STATS: (2100 meters !? center 270, 460, 220): 64 - NO, 128 - 15, 256 - 7.5
	//	BB: (-712.772  -482.46  -586.02) - (1252.08 1394.97 1025.27)


	std::string voxel_identifier = "/VOXELS_" + std::to_string(grid_width_voxels);


	std::string output_folder = GetSDF_MeshDatasetsPath() + voxel_identifier + sequence_file_identifier;

	std::string output_mesh_tag = output_folder + "/FRAME";

	void ConvertSequence();

public:

	void run(int argc, char** argv);
};