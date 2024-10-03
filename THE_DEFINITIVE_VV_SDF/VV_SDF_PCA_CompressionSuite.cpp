#include "VV_SDF_PCA_CompressionSuite.h"

bool VV_SDF_PCA_CompressionSuite::InitializeCompressor()
{
	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	if (!compressor.PreInitialization(gds, block_size, pca_reach, significant_value_ratio, max_allowed_components))
	{
		return false;
	}

	return true;
}

void VV_SDF_PCA_CompressionSuite::TestCreateIntermediary()
{
	if (!InitializeCompressor())
	{
		return;
	}

	if (!compressor.SaveIntermediaryFile(test_seq_folder, intermediary_file, shell_size, mesh_maximum_artifact_size))
	{
		return;
	}
}

void VV_SDF_PCA_CompressionSuite::TestRecreateMeshesFromIntermediary()
{
	compressor.ReconstructMeshesFromIntermediary(intermediary_file, intermediary_reconstruction_folder, intermediary_reconstruction_tag, 6);
}

void VV_SDF_PCA_CompressionSuite::TestCreatePCA()
{
	compressor.SavePCA_File(intermediary_file, pca_file);
}

void VV_SDF_PCA_CompressionSuite::TestSingleReconstruction()
{
	compressor.ReconstructMeshesFromPCA(pca_file, reconstruction_folder, reconstruction_tag, 6);
}

void VV_SDF_PCA_CompressionSuite::TestSpecificSegmentBeforeAndAfterRLE(Eigen::Vector4i location, Eigen::Vector4i span)
{
	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / grid_width_voxels;

	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	compressor.TestSpecificSection(
		gds,
		test_seq_folder, location,
		block_size, shell_size, pca_reach,
		significant_value_ratio, max_allowed_components, mesh_maximum_artifact_size);
}

void VV_SDF_PCA_CompressionSuite::run(int argc, char** argv)
{
	//TestSingleSequence();

	TestCreateIntermediary();
	//
	//system("pause");
	//
	////TestRecreateMeshesFromIntermediary();
	//
	TestCreatePCA();
	//
	//system("pause");

	TestSingleReconstruction();

	//TestSpecificSegmentBeforeAndAfterRLE(Eigen::Vector4i(2, 0, 4, 0), Eigen::Vector4i());
}
