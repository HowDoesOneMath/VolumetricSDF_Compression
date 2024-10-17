#pragma once

#include "TestSuite.h"

#include "SequenceFilePathUniversal.h"
#include "SequenceFinder.h"

#include "PointCloudGenerator.h"
#include "MeshEvaluationMetrics.h"

#include <vector>
#include <string>
#include <fstream>

#include <chrono>

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

	std::ofstream to_write;

	std::vector<ErrorMetricSet> em_set = {
		ErrorMetricSet(GetReconstructionPath() + "/SIR_FREDRICK_STATS.txt", GetDatasetsPath() + "/SIR_FREDRICK", {
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_0/QUANT_11_10_8/SIR_FREDRICK",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_7/QUANT_11_10_8/SIR_FREDRICK",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_10/QUANT_11_10_8/SIR_FREDRICK",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM4LOOP1/SIR_FREDRICK",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM10LOOP2/SIR_FREDRICK",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM16LOOP2/SIR_FREDRICK",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM40LOOP3/SIR_FREDRICK",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM64LOOP3/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/SIR_FREDRICK",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/BASKETBALL_STATS.txt", GetDatasetsPath() + "/Basketball", {
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_0/QUANT_11_10_8/BASKETBALL",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_7/QUANT_11_10_8/BASKETBALL",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_10/QUANT_11_10_8/BASKETBALL",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM4LOOP1/BASKETBALL",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM10LOOP2/BASKETBALL",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM16LOOP2/BASKETBALL",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM40LOOP3/BASKETBALL",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM64LOOP3/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/BASKETBALL",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/BASKETBALL",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/RAFA_STATS.txt", GetDatasetsPath() + "/RAFA", {
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_0/QUANT_11_10_8/RAFA",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_7/QUANT_11_10_8/RAFA",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_10/QUANT_11_10_8/RAFA",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM4LOOP1/RAFA",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM10LOOP2/RAFA",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM16LOOP2/RAFA",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM40LOOP3/RAFA",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM64LOOP3/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/RAFA",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/RAFA",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/LEVI_STATS.txt", GetDatasetsPath() + "/LEVI", {
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_0/QUANT_11_10_8/LEVI",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_7/QUANT_11_10_8/LEVI",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_10/QUANT_11_10_8/LEVI",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM4LOOP1/LEVI",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM10LOOP2/LEVI",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM16LOOP2/LEVI",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM40LOOP3/LEVI",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM64LOOP3/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/LEVI",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/LEVI",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-2PUNCH_STATS.txt", GetDatasetsPath() + "/AB-2punch", {
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_0/QUANT_11_10_8/AB-2PUNCH",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_7/QUANT_11_10_8/AB-2PUNCH",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_10/QUANT_11_10_8/AB-2PUNCH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM4LOOP1/AB-2PUNCH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM10LOOP2/AB-2PUNCH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM16LOOP2/AB-2PUNCH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM40LOOP3/AB-2PUNCH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM64LOOP3/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-2PUNCH",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-DODGE_STATS.txt", GetDatasetsPath() + "/AB-dodgeLeft", {
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_0/QUANT_11_10_8/AB-DODGE",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_7/QUANT_11_10_8/AB-DODGE",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_10/QUANT_11_10_8/AB-DODGE",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM4LOOP1/AB-DODGE",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM10LOOP2/AB-DODGE",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM16LOOP2/AB-DODGE",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM40LOOP3/AB-DODGE",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM64LOOP3/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/AB-DODGE",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-DODGE",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-DEATH_STATS.txt", GetDatasetsPath() + "/AB-death", {
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_0/QUANT_11_10_8/AB-DEATH",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_7/QUANT_11_10_8/AB-DEATH",
				GetReconstructionPath() + "/_DRACO/COMPRESSION_SPEED_10/QUANT_11_10_8/AB-DEATH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM4LOOP1/AB-DEATH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM10LOOP2/AB-DEATH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM16LOOP2/AB-DEATH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM40LOOP3/AB-DEATH",
				GetReconstructionPath() + "/_VSMC/DRACO_COMPRESSION_0/DECIM64LOOP3/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/AB-DEATH",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-DEATH",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/SIR_FREDRICK_STATS_COMPARE_SDF_64.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_64/SIR_FREDRICK", {
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/SIR_FREDRICK",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/SIR_FREDRICK_STATS_COMPARE_SDF_256.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_256/SIR_FREDRICK", {
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/SIR_FREDRICK",
			}
		),

		//DOES NOT EXIST
		//ErrorMetricSet(GetReconstructionPath() + "/BASKETBALL_STATS_COMPARE_SDF_64.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_64/BASKETBALL", {
		//		GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/BASKETBALL",
		//	}
		//),

		ErrorMetricSet(GetReconstructionPath() + "/BASKETBALL_STATS_COMPARE_SDF_256.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_256/BASKETBALL", {
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/BASKETBALL",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/RAFA_STATS_COMPARE_SDF_64.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_64/RAFA", {
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/RAFA",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/RAFA_STATS_COMPARE_SDF_256.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_256/RAFA", {
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/RAFA",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/LEVI_STATS_COMPARE_SDF_64.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_64/LEVI", {
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/LEVI",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/LEVI_STATS_COMPARE_SDF_256.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_256/LEVI", {
			GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/LEVI",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-2PUNCH_STATS_COMPARE_SDF_64.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_64/AB-2PUNCH", {
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/AB-2PUNCH",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-2PUNCH_STATS_COMPARE_SDF_256.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_256/AB-2PUNCH", {
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/AB-2PUNCH",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-DODGE_STATS_COMPARE_SDF_64.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_64/AB-DODGE", {
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/AB-DODGE",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-DODGE_STATS_COMPARE_SDF_256.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_256/AB-DODGE", {
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/AB-DODGE",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-DEATH_STATS_COMPARE_SDF_64.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_64/AB-DEATH", {
				GetReconstructionPath() + "/TSVD/VOXELS_64/BATCH_256/SIGNIFICANCE_90/AB-DEATH",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-DEATH_STATS_COMPARE_SDF_256.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_256/AB-DEATH", {
				GetReconstructionPath() + "/TSVD/VOXELS_256/BATCH_256/SIGNIFICANCE_90/AB-DEATH",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/SIR_FREDRICK_STATS_COMPARE_SDF_128.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_128/SIR_FREDRICK", {
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/SIR_FREDRICK",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/SIR_FREDRICK",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/BASKETBALL_STATS_COMPARE_SDF_128.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_128/BASKETBALL", {
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/BASKETBALL",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/BASKETBALL",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/BASKETBALL",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/RAFA_STATS_COMPARE_SDF_128.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_128/RAFA", {
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/RAFA",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/RAFA",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/RAFA",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/LEVI_STATS_COMPARE_SDF_128.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_128/LEVI", {
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/LEVI",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/LEVI",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/LEVI",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-2PUNCH_STATS_COMPARE_SDF_128.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_128/AB-2PUNCH", {
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/AB-2PUNCH",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-2PUNCH",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-DODGE_STATS_COMPARE_SDF_128.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_128/AB-DODGE", {
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/AB-DODGE",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/AB-DODGE",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-DODGE",
			}
		),

		ErrorMetricSet(GetReconstructionPath() + "/AB-DEATH_STATS_COMPARE_SDF_128.txt", GetSDF_MeshDatasetsPath() + "/VOXELS_128/AB-DEATH", {
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_128/SIGNIFICANCE_90/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_64/SIGNIFICANCE_90/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_99/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_95/AB-DEATH",
				GetReconstructionPath() + "/TSVD/VOXELS_128/BATCH_256/SIGNIFICANCE_80/AB-DEATH",
				GetReconstructionPath() + "/TSVD_PLUS_SIGNS/VOXELS_128/BATCH_256/SIGNIFICANCE_90/AB-DEATH",
			}
		),
	};

	size_t point_cloud_density;

	void RunAllSets();

	std::pair<Eigen::Vector3d, Eigen::Vector3d> GetBoundingBox(std::vector<std::string> &input_meshes);
public:

	void run(int argc, char** argv);
};