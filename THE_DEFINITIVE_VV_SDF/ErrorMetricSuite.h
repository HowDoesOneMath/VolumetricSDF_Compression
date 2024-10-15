#pragma once

#include "TestSuite.h"

#include "SequenceFilePathUniversal.h"
#include "SequenceFinder.h"

#include "PointCloudGenerator.h"
#include "MeshEvaluationMetrics.h"

#include <vector>
#include <string>
#include <fstream>

class ErrorMetricSuite : public TestSuite
{
	class ErrorMetricSet
	{
	public:
		ErrorMetricSet(std::string file_name, std::string original_sequence, std::vector<std::string> to_compare) {
			this->file_name = file_name;
			this->original_sequence = original_sequence;
			this->to_compare = to_compare;
		}

		std::string file_name;
		std::string original_sequence;
		
		std::vector<std::string> to_compare;
	};

	SequenceFinder comparator_seq;

	SequenceFinderDetails mesh_sf_original = SequenceFinderDetails("Mesh_Orig", ".obj");
	SequenceFinderDetails mesh_sf_to_compare = SequenceFinderDetails("Mesh_Comp", ".obj");

	PointCloudGenerator pcg;
	double point_cloud_spacing = 0.005;
	double epsilon_scalar = 0.00001;
	Eigen::Vector3d epsilon_vector = epsilon_scalar * Eigen::Vector3d::Ones();

	MeshEvaluationMetrics mem;
	std::ofstream to_write;

	std::vector<ErrorMetricSet> em_set = {
		ErrorMetricSet(GetReconstructionPath() + "/VSMC/SIR_FREDRICK_STATS.txt", GetDatasetsPath() + "/SIR_FREDRICK", {
				GetReconstructionPath() + "/VSMC/DRACO_COMPRESSION_0/DECIM4LOOP1/SIR_FREDRICK",
				GetReconstructionPath() + "/VSMC/DRACO_COMPRESSION_0/DECIM10LOOP2/SIR_FREDRICK",
				GetReconstructionPath() + "/VSMC/DRACO_COMPRESSION_0/DECIM16LOOP2/SIR_FREDRICK",
				GetReconstructionPath() + "/VSMC/DRACO_COMPRESSION_0/DECIM40LOOP3/SIR_FREDRICK",
				GetReconstructionPath() + "/VSMC/DRACO_COMPRESSION_0/DECIM64LOOP3/SIR_FREDRICK",
			}
		),


	};

	size_t point_cloud_density;

	void RunAllSets();

	std::pair<Eigen::Vector3d, Eigen::Vector3d> GetBoundingBox(std::vector<std::string> &input_meshes);
public:

	void run(int argc, char** argv);
};