#include "PCA_Suite.h"

bool PCA_Suite::InitializeSDF()
{
	gds.dim_x = square_grid_size;
	gds.dim_y = 2 * square_grid_size;
	gds.dim_z = square_grid_size;

	gds.unit_length = grid_size / (square_grid_size - 1);
	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	if (!sdf.InitializeGrid(gds))
	{
		return false;
	}

	return true;
}

bool PCA_Suite::LoadSDF(std::string file_name)
{
	if (!mesh.ReadOBJ(file_name))
	{
		return false;
	}

	sdf.CastMeshUnsignedDistance(&mesh, buffer_size);

	return true;
}

void PCA_Suite::SampleGridWhole()
{
	//Eigen::Vector2i estimated_uv_divisions = Eigen::Vector2i(30 * square_grid_size, square_grid_size);
	auto first_mesh = sdf.ExtractMeshQuantized();// estimated_uv_divisions, 0.126);

	AddChunksToEncoder(sdf, grid_block_length, grid_block_length, grid_block_length);

	std::cout << "Conducting SVD... " << std::endl;

	pca_enc.ConductSVD();

	Eigen::VectorXd values = pca_enc.GetValues();

	Eigen::MatrixXd culled_u = pca_enc.GetMatrixU();
	Eigen::MatrixXd culled_v = pca_enc.GetMatrixV();

	size_t u_len = culled_u.rows();
	
	size_t all_values;
	size_t good_values = 0;
	for (all_values = 0; all_values < values.size(); ++all_values)
	{
		if (values[all_values] >= PCA_cull_threshold)
		{
			++good_values;
			continue;
		}

		break;
	}

	if (good_values == 0)
	{
		std::cout << "ERROR! No good values!" << std::endl;
		return;
	}

	good_values = 50;

	Eigen::MatrixXd sub_u = culled_u.block(0, 0, culled_u.rows(), good_values);
	Eigen::VectorXd sub_sigma = values.block(0, 0, good_values, 1);
	Eigen::MatrixXd sub_v = culled_v.block(0, 0, culled_v.rows(), good_values);

	std::cout << "Good Values: " << good_values << std::endl;

	std::cout << "Recreating PCA with reduced matrices... " << std::endl;

	pca_enc.Recreate(sub_u, sub_sigma, sub_v);

	std::cout << "Replacing chunks... " << std::endl;

	ExtractChunksFromEncoder(sdf, grid_block_length, grid_block_length, grid_block_length);

	std::cout << "Extracting mesh..." << std::endl;

	auto to_save = sdf.ExtractMeshQuantized();// estimated_uv_divisions, 0.126);

	to_save->ConcatenateMesh(*first_mesh, mesh_offset);
	to_save->ConcatenateMesh(mesh, 2 * mesh_offset);

	std::cout << "Saving..." << std::endl;

	to_save->WriteOBJ(test_PCA_output_mesh);

	pca_enc.CleanUpData();
}

void PCA_Suite::SaveGridDividedRLE()
{
	if (!InitializeSDF())
	{
		std::cout << "ERROR - could not initialize SDF!" << std::endl;
		return;
	}

	sf.FindFiles(test_sequence, mesh_sf);

	std::vector<size_t> file_locations;
	size_t current_location = 0;

	std::vector<unsigned char> block_buffer;
	block_buffer.resize(grid_block_length * grid_block_length * grid_block_length);

	size_t grid_partitions_single_axis = square_grid_size / grid_block_length;
	size_t grid_partitions_total = grid_partitions_single_axis * grid_partitions_single_axis * grid_partitions_single_axis;

	size_t total_rle_segments = grid_partitions_total * sf.files[mesh_sf.key].size();
	file_locations.resize(total_rle_segments);


	piecewise_rle.OpenWriteBuffer(test_RLE_output);

	piecewise_rle.WriteObjectToBuffer(square_grid_size);
	piecewise_rle.WriteObjectToBuffer(grid_block_length);

	size_t file_locations_start = piecewise_rle.GetWriterLocation();
	for (size_t i = 0; i < total_rle_segments; ++i)
	{
		piecewise_rle.WriteObjectToBuffer(total_rle_segments);
	}


	for (int i = 0; i < sf.files[mesh_sf.key].size(); ++i)
	{
		if (!LoadSDF(sf.files[mesh_sf.key][i]))
		{
			std::cout << "ERROR: mesh could not be loaded: " << sf.files[mesh_sf.key][i] << std::endl;
			return;
		}

		for (size_t x = 0; x < square_grid_size; x += grid_block_length)
		{
			for (size_t y = 0; y < square_grid_size; y += grid_block_length)
			{
				for (size_t z = 0; z < square_grid_size; z += grid_block_length)
				{
					sdf.ExtractBlock(x, y, z, grid_block_length, grid_block_length, grid_block_length, block_buffer.data());

					rle.GetRunLength(block_buffer.data(), block_buffer.size());

					file_locations[current_location] = piecewise_rle.GetWriterLocation();
					++current_location;

					for (int j = 0; j < rle.runs.size(); ++j)
					{
						piecewise_rle.WriteObjectToBuffer(rle.lengths[j]);
					}
					for (int j = 0; j < rle.runs.size(); ++j)
					{
						piecewise_rle.WriteObjectToBuffer(rle.runs[j]);
					}
				}
			}
		}
	}


	size_t writer_eof = piecewise_rle.GetWriterLocation();

	piecewise_rle.SetWriterLocation(file_locations_start);
	for (size_t i = 0; i < total_rle_segments; ++i)
	{
		piecewise_rle.WriteObjectToBuffer(file_locations[i]);
	}

	piecewise_rle.SetWriterLocation(writer_eof);
	piecewise_rle.CloseWriteBuffer();
}

void PCA_Suite::AddChunksToEncoder(VV_TSDF &to_encode, size_t block_length_x, size_t block_length_y, size_t block_length_z)
{
	size_t partitions_x = sdf.GetDimX() / block_length_x;
	size_t partitions_y = sdf.GetDimY() / block_length_y;
	size_t partitions_z = sdf.GetDimZ() / block_length_z;
	
	size_t grid_partitions_total = partitions_x * partitions_y * partitions_z;
	size_t block_length_total = block_length_x * block_length_y * block_length_z;

	pca_enc.Initialize(grid_partitions_total, block_length_total);


	std::vector<unsigned char> copy_array(block_length_total);


	size_t span_y = to_encode.GetSpanY();
	size_t span_x = to_encode.GetSpanX();

	size_t pca_span_y = block_length_z;
	size_t pca_span_x = block_length_y * block_length_z;

	size_t pca_row = 0;

	for (size_t x = 0; x < to_encode.GetDimX(); x += block_length_x)
	{
		for (size_t y = 0; y < to_encode.GetDimY(); y += block_length_y)
		{
			for (size_t z = 0; z < to_encode.GetDimZ(); z += block_length_z, ++pca_row)
			{
				to_encode.ExtractBlock(x, y, z, block_length_x, block_length_y, block_length_z, copy_array.data());

				pca_enc.AddData(pca_row, copy_array.data());
			}
		}
	}
}

void PCA_Suite::ExtractChunksFromEncoder(VV_TSDF& to_decode, size_t block_length_x, size_t block_length_y, size_t block_length_z)
{
	size_t block_length_total = block_length_x * block_length_y * block_length_z;

	std::vector<unsigned char> copy_array(block_length_total);

	size_t span_y = to_decode.GetSpanY();
	size_t span_x = to_decode.GetSpanX();

	size_t pca_span_y = block_length_z;
	size_t pca_span_x = block_length_y * block_length_z;

	size_t pca_row = 0;

	std::cout << "SDF dims: " <<
		to_decode.GetDimX() << ", " <<
		to_decode.GetDimY() << ", " <<
		to_decode.GetDimZ() << std::endl;

	for (size_t x = 0; x < to_decode.GetDimX(); x += block_length_x)
	{
		for (size_t y = 0; y < to_decode.GetDimY(); y += block_length_y)
		{
			for (size_t z = 0; z < to_decode.GetDimZ(); z += block_length_z, ++pca_row)
			{
				pca_enc.ExtractDataAndClamp(pca_row, copy_array.data(), to_decode.min_c, to_decode.max_c);

				to_decode.InsertBlock(x, y, z, block_length_x, block_length_y, block_length_z, copy_array.data());
			}
		}
	}


}

void PCA_Suite::RedundantSVD()
{
	if (!InitializeSDF())
	{
		std::cout << "ERROR - could not initialize SDF!" << std::endl;
		return;
	}

	if (!LoadSDF(test_singular_mesh))
	{
		std::cout << "Error! Couldn't load mesh for svd!" << std::endl;
		return;
	}

	std::cout << "Saving base mesh... " << std::endl;

	//Eigen::Vector2i estimated_uv_divisions = Eigen::Vector2i(30 * square_grid_size, square_grid_size);
	sdf.ExtractMeshQuantized();// estimated_uv_divisions, 0.126)->WriteOBJ(test_before_PCA_output_mesh);

	std::cout << "Mesh loaded, getting blocks... " << std::endl;

	AddChunksToEncoder(sdf, grid_block_length, grid_block_length, grid_block_length);

	std::cout << "pca_mat_size: " << pca_enc.GetMat()->rows() << ", " << pca_enc.GetMat()->cols() << std::endl;

	std::cout << "Conducting SVD... " << std::endl;

	pca_enc.ConductSVD();

	std::cout << "Conducting Reverse SVD... " << std::endl;

	Eigen::MatrixXd U = pca_enc.GetMatrixU();
	Eigen::MatrixXd V = pca_enc.GetMatrixV();

	Eigen::VectorXd diag = pca_enc.GetValues();

	//std::cout << diag.size() << ":\t\t" << diag.transpose() << std::endl;
	std::cout << 
		"U matrix dims: " << U.rows() << ", " << U.cols() << ":\t\t" << 
		"V matrix dims: " << V.rows() << ", " << V.cols() << std::endl;

	pca_enc.Recreate(U, diag, V);

	std::cout << "Emplacing chunks in sdf..." << std::endl;

	ExtractChunksFromEncoder(sdf, grid_block_length, grid_block_length, grid_block_length);

	std::cout << "Extracting mesh..." << std::endl;

	auto to_save = sdf.ExtractMeshQuantized();// estimated_uv_divisions, 0.126);

	std::cout << "Saving..." << std::endl;

	to_save->WriteOBJ(test_PCA_output_mesh);
}

void PCA_Suite::MiniTestSVD()
{
	const int w = 3;
	const int h = 4;

	double mini_pca_test[w * h] = {
		0, 1, 0,
		-1, 2, 3,
		0.5, 6, 199,
		0.1, 0.01, 0.001
	};

	pca_enc.Initialize(h, w);

	for (size_t i = 0; i < h; ++i)
	{
		pca_enc.AddData(i, &(mini_pca_test[w * i]));
	}

	std::cout << "Matrix pre-calcs:\n" << *(pca_enc.GetMat()) << std::endl;

	pca_enc.ConductSVD();

	Eigen::MatrixXd U = pca_enc.GetMatrixU();
	Eigen::MatrixXd V = pca_enc.GetMatrixV();

	Eigen::VectorXd diag = pca_enc.GetValues();

	pca_enc.Recreate(U, diag, V);


	std::cout << "Matrix post-calcs:\n" << *(pca_enc.GetMat()) << std::endl;
}

void PCA_Suite::DummyFillGrid()
{
	InitializeSDF();

	size_t partitions_x = sdf.GetDimX() / grid_block_length;
	size_t partitions_y = sdf.GetDimY() / grid_block_length;
	size_t partitions_z = sdf.GetDimZ() / grid_block_length;

	size_t grid_partitions_total = partitions_x * partitions_y * partitions_z;
	size_t block_length_total = grid_block_length * grid_block_length * grid_block_length;

	pca_enc.Initialize(grid_partitions_total, block_length_total);

	std::vector<unsigned char> copy_array(block_length_total);

	size_t span_y = sdf.GetSpanY();
	size_t span_x = sdf.GetSpanX();

	size_t pca_span_y = grid_block_length;
	size_t pca_span_x = grid_block_length * grid_block_length;

	size_t pca_row = 0;

	auto sdf_pointer = sdf.GetQuantizedGridPointer();

	for (size_t i = 0; i < sdf.GetQuantizedGridLength(); ++i)
	{
		sdf_pointer[i] = i % 100;
	}

	sdf.ExtractBlock(0, 0, 0, grid_block_length, grid_block_length, grid_block_length, copy_array.data());

	std::cout << "\nArray: ";

	for (size_t i = 0; i < copy_array.size(); ++i)
	{
		std::cout << "\t" << (int)copy_array[i];
	}

	std::cout << std::endl;
}

void PCA_Suite::run(int argc, char** argv)
{
	InitializeSDF();
	LoadSDF(test_singular_mesh);
	
	std::cout << "Loaded SDF..." << std::endl;
	
	SampleGridWhole();

	//SaveGridDividedRLE();

	//RedundantSVD();

	//DummyFillGrid();

	//MiniTestSVD();
}
