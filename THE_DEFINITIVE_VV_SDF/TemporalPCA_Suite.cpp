#include "TemporalPCA_Suite.h"

void TemporalPCA_Suite::LoadSequences()
{
	sf.FindFiles(sequence_folder, mesh_sf);
}

void TemporalPCA_Suite::InitializeSDF()
{
	gds.center_x = center.x();
	gds.center_y = center.y();
	gds.center_z = center.z();

	gds.dim_x = grid_width_voxels;
	gds.dim_y = 2 * grid_width_voxels;
	gds.dim_z = grid_width_voxels;

	gds.unit_length = grid_width_meters / (grid_width_voxels - 1);

	sdf.InitializeGrid(gds);
}

void TemporalPCA_Suite::InitializePCA_Encoder()
{
	pca_enc.Initialize(span_size.x() * span_size.y() * span_size.z(), block_size.x() * block_size.y() * block_size.z());
}

void TemporalPCA_Suite::ReadAndCleanMesh(std::string to_read, VV_Mesh* mesh, double artifact_size)
{
	mesh->ReadOBJ(to_read);

	mp.CreateUnionFindPartitions(*mesh);
	mp.NegateInsignificantPartitions(*mesh, artifact_size);
	mesh->ClearNegativeTriangles();
	mp.ClearPartitions();
}

void TemporalPCA_Suite::LoadPCA_Matrix(PCA_Encoder* enc, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size)
{
	Eigen::Vector3i start = section_location;
	Eigen::Vector3i end = start + section_size;

	size_t pca_row = 0;

	std::vector<unsigned char> sdf_data_chunk;
	//std::vector<double> sdf_data_chunk;
	sdf_data_chunk.resize(block_size.x() * block_size.y() * block_size.z());

	for (size_t x = start.x(); x < end.x(); x += block_size.x())
	{
		for (size_t y = start.y(); y < end.y(); y += block_size.y())
		{
			for (size_t z = start.z(); z < end.z(); z += block_size.z(), ++pca_row)
			{
				sdf->ExtractBlock(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockMortonOrder(x, y, z, sdf_data_chunk.size(), sdf_data_chunk.data());
				//sdf->ExtractBlockS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockUnquantized(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockUnquantizedS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());

				enc->AddData(pca_row, sdf_data_chunk.data());
			}
		}
	}
}

void TemporalPCA_Suite::LoadPCA_MatrixInBlocks(PCA_Encoder* enc, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size)
{
	Eigen::Vector3i start = section_location;
	Eigen::Vector3i end = start + section_size;

	size_t pca_row = 0;

	std::vector<unsigned char> sdf_data_chunk;
	//std::vector<double> sdf_data_chunk;
	sdf_data_chunk.resize(block_size.x() * block_size.y() * block_size.z());

	size_t blockwise_c = 0;
	size_t blockwise_c_step = block_size.x();
	size_t blockwise_r = 0;
	size_t blockwise_r_step = block_size.z() * block_size.y();

	for (size_t x = start.x(); x < end.x(); x += block_size.x())
	{
		for (size_t y = start.y(); y < end.y(); y += block_size.y())
		{
			for (size_t z = start.z(); z < end.z(); z += block_size.z(), ++pca_row)
			{
				sdf->ExtractBlock(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockMortonOrder(x, y, z, sdf_data_chunk.size(), sdf_data_chunk.data());
				//sdf->ExtractBlockS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockUnquantized(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockUnquantizedS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());

				//std::cout << blockwise_r << ", " << blockwise_c << std::endl;

				enc->AddDataBlock(blockwise_r, blockwise_c, blockwise_r + blockwise_r_step, blockwise_c + blockwise_c_step, sdf_data_chunk.data());

				blockwise_r += blockwise_r_step;
				while (blockwise_r >= enc->GetMat()->rows())
				{
					blockwise_r -= enc->GetMat()->rows();
					blockwise_c += blockwise_c_step;
				}
			}
		}
	}
}

void TemporalPCA_Suite::LoadMatrix(Eigen::MatrixXd& mat, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size)
{
	Eigen::Vector3i start = section_location;
	Eigen::Vector3i end = start + section_size;

	size_t mat_row = 0;

	std::vector<unsigned char> sdf_data_chunk;
	//std::vector<double> sdf_data_chunk;
	sdf_data_chunk.resize(block_size.x() * block_size.y() * block_size.z());

	for (size_t x = start.x(); x < end.x(); x += block_size.x())
	{
		for (size_t y = start.y(); y < end.y(); y += block_size.y())
		{
			for (size_t z = start.z(); z < end.z(); z += block_size.z(), ++mat_row)
			{
				sdf->ExtractBlock(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockMortonOrder(x, y, z, sdf_data_chunk.size(), sdf_data_chunk.data());
				//sdf->ExtractBlockS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockUnquantized(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->ExtractBlockUnquantizedS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());

				for (size_t i = 0; i < mat.cols(); ++i)
				{
					mat(mat_row, i) = sdf_data_chunk[i];
				}
			}
		}
	}
}

void TemporalPCA_Suite::EmplaceMatrix(Eigen::MatrixXd& mat, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size)
{
	Eigen::Vector3i start = section_location;
	Eigen::Vector3i end = start + section_size;

	size_t mat_row = 0;

	std::vector<unsigned char> sdf_data_chunk;
	//std::vector<double> sdf_data_chunk;
	sdf_data_chunk.resize(block_size.x() * block_size.y() * block_size.z());

	for (size_t x = start.x(); x < end.x(); x += block_size.x())
	{
		for (size_t y = start.y(); y < end.y(); y += block_size.y())
		{
			for (size_t z = start.z(); z < end.z(); z += block_size.z(), ++mat_row)
			{
				for (size_t i = 0; i < mat.cols(); ++i)
				{
					sdf_data_chunk[i] = std::clamp(mat(mat_row, i), (double)sdf->min_c + 0.1, (double)sdf->max_c + 0.1);
					//sdf_data_chunk[i] = mat(mat_row, i);
				}

				sdf->InsertBlock(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->InsertBlockMortonOrder(x, y, z, sdf_data_chunk.size(), sdf_data_chunk.data());
				//sdf->InsertBlockS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->InsertBlockUnquantized(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->InsertBlockUnquantizedS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
			}
		}
	}
}

void TemporalPCA_Suite::EmplacePCA_MatrixInBlocks(PCA_Encoder* enc, VV_TSDF* sdf, Eigen::Vector3i section_location, Eigen::Vector3i section_size, Eigen::Vector3i block_size)
{
	Eigen::Vector3i start = section_location;
	Eigen::Vector3i end = start + section_size;

	size_t mat_row = 0;

	std::vector<unsigned char> sdf_data_chunk;
	//std::vector<double> sdf_data_chunk;
	sdf_data_chunk.resize(block_size.x() * block_size.y() * block_size.z());

	size_t blockwise_c = 0;
	size_t blockwise_c_step = block_size.x();
	size_t blockwise_r = 0;
	size_t blockwise_r_step = block_size.z() * block_size.y();

	for (size_t x = start.x(); x < end.x(); x += block_size.x())
	{
		for (size_t y = start.y(); y < end.y(); y += block_size.y())
		{
			for (size_t z = start.z(); z < end.z(); z += block_size.z(), ++mat_row)
			{
				enc->ExtractDataBlockAndClamp(blockwise_r, blockwise_c, blockwise_r + blockwise_r_step, blockwise_c + blockwise_c_step,
					sdf_data_chunk.data(), (double)sdf->min_c + 0.1, (double)sdf->max_c + 0.1);

				sdf->InsertBlock(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->InsertBlockMortonOrder(x, y, z, sdf_data_chunk.size(), sdf_data_chunk.data());
				//sdf->InsertBlockS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->InsertBlockUnquantized(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());
				//sdf->InsertBlockUnquantizedS_Traversal(x, y, z, block_size.x(), block_size.y(), block_size.z(), sdf_data_chunk.data());

				blockwise_r += blockwise_r_step;
				while (blockwise_r >= enc->GetMat()->rows())
				{
					blockwise_r -= enc->GetMat()->rows();
					blockwise_c += blockwise_c_step;
				}
			}
		}
	}
}

void TemporalPCA_Suite::CheckAdjacentFrames()
{
	size_t to_print = 12;

	Eigen::Vector3i pca_chunk_dims = block_size.cwiseProduct(span_size);
	Eigen::Vector3i section_count = Eigen::Vector3i(
		sdf.GetDimX() / pca_chunk_dims.x(),
		sdf.GetDimY() / pca_chunk_dims.y(),
		sdf.GetDimZ() / pca_chunk_dims.z()
	);

	auto file_vector = sf.GetFileVector(mesh_sf.key);

	std::cout << "Loading mesh... " << std::endl;

	ReadAndCleanMesh((*file_vector)[0], &input_mesh, artifact_size);
	sdf.CastMeshUnsignedDistance(&input_mesh, sdf_buffer_distance);
	LoadPCA_Matrix(&pca_enc, &sdf, Eigen::Vector3i(0, 0, 0), pca_chunk_dims, block_size);

	std::cout << "Conducting SVD..." << std::endl;
	pca_enc.ConductSVD();

	std::cout << "\n" << pca_enc.GetValues().asDiagonal().toDenseMatrix().block(0, 0, to_print, to_print) << "\n" << std::endl;

	std::cout << "Getting Inverted Matrices... " << std::endl;

	Eigen::MatrixXd u = pca_enc.GetMatrixU();
	Eigen::MatrixXd v = pca_enc.GetMatrixV();

	Eigen::MatrixXd inv_u = u.inverse();
	Eigen::MatrixXd inv_v = v.transpose().inverse();

	std::cout << "Reading second mesh... " << std::endl;

	ReadAndCleanMesh((*file_vector)[1], &input_mesh, artifact_size);
	sdf.CastMeshUnsignedDistance(&input_mesh, sdf_buffer_distance);
	LoadPCA_Matrix(&pca_enc, &sdf, Eigen::Vector3i(0, 0, 0), pca_chunk_dims, block_size);

	std::cout << "Applying U and V inverse..." << std::endl;

	Eigen::MatrixXd new_diag = inv_u * (*pca_enc.GetMat()) * inv_v;

	std::cout << "\n" << new_diag.block(0, 0, to_print, to_print) << "\n" << std::endl;

	//for (size_t x = 0; x < gds.dim_x; x += pca_chunk_dims.x())
	//{
	//	for (size_t y = 0; y < gds.dim_y; y += pca_chunk_dims.y())
	//	{
	//		for (size_t z = 0; z < gds.dim_z; z += pca_chunk_dims.z())
	//		{
	//			LoadPCA_Matrix(&pca_enc, &sdf, Eigen::Vector3i(x, y, z), pca_chunk_dims, block_size);
	//		}
	//	}
	//}

	//for (int i = 0; i < file_vector->size(); ++i)
	//{
	//	
	//}
}

void TemporalPCA_Suite::SaveAdjacentFramesWithPCA_Reuse(size_t frame_cap)
{
	Eigen::MatrixXd temp_mat(pca_enc.GetMat()->rows(), pca_enc.GetMat()->cols());
	Eigen::MatrixXd emplacing_mat(pca_enc.GetMat()->rows(), pca_enc.GetMat()->cols());

	std::vector<Eigen::VectorXd> values;
	std::vector<Eigen::MatrixXd> u_mats;
	std::vector<Eigen::MatrixXd> u_inv_mats;
	std::vector<Eigen::MatrixXd> v_mats;
	std::vector<Eigen::MatrixXd> v_inv_mats;

	std::vector<size_t> out_of_order;
	out_of_order.resize(pca_enc.GetMat()->rows());

	Eigen::Vector3i pca_chunk_dims = block_size.cwiseProduct(span_size);
	Eigen::Vector3i section_count = Eigen::Vector3i(
		sdf.GetDimX() / pca_chunk_dims.x(),
		sdf.GetDimY() / pca_chunk_dims.y(),
		sdf.GetDimZ() / pca_chunk_dims.z()
	);

	values.resize(section_count.x() * section_count.y() * section_count.z());
	u_mats.resize(values.size());
	v_mats.resize(values.size());
	u_inv_mats.resize(values.size());
	v_inv_mats.resize(values.size());

	auto file_vector = sf.GetFileVector(mesh_sf.key);

	size_t total_significant_values = 0;
	size_t zero_chunks = 0;
	size_t lz_size = 0;
	size_t total_pca_size = 0;
	size_t original_size = 0;
	size_t significant_values_this_frame;

	std::vector<unsigned char> lze_data;
	std::vector<unsigned char> sign_data;

	size_t frame_count = std::min(file_vector->size(), frame_cap);

	//std::vector<unsigned char> iframe;
	//iframe.resize(sdf.GetQuantizedGridLength());

	for (size_t i = 0; i < frame_count; ++i)
	{
		size_t modulo_frame = i % group_size;
		size_t section_mux = 0;

		std::string save_name = grouping_output_folder + "/" + output_mesh_tag + GetNumberFixedLength(i, 6) + output_mesh_ext;

		std::cout << "Loading mesh " << i << " ... " << std::endl;

		ReadAndCleanMesh((*file_vector)[i], &input_mesh, artifact_size);
		sdf.CastMeshUnsignedDistance(&input_mesh, sdf_buffer_distance);

		//sdf.ClampUnquantizedGrid();
		//sdf.HarvestSigns(sign_data);

		auto original_mesh = sdf.ExtractMeshQuantized();// Eigen::Vector2i(1024, 1024), 0.1);
		original_size += sdf.GetQuantizedGridLength();

		lze.CompressData(sdf.GetQuantizedGridPointer(), sdf.GetQuantizedGridLength(), lze_data);
		lz_size += lze_data.size();


		Eigen::Vector3i section;

		//if (modulo_frame != 0)
		//{
		//	sdf.SubtractFromGrid(iframe.data());
		//}

		significant_values_this_frame = 0;

		for (section.x() = 0; section.x() < gds.dim_x; section.x() += pca_chunk_dims.x())
		{
			for (section.y() = 0; section.y() < gds.dim_y; section.y() += pca_chunk_dims.y())
			{
				for (section.z() = 0; section.z() < gds.dim_z; section.z() += pca_chunk_dims.z(), ++section_mux)
				{
					std::cout << "\tSection " << section.transpose() << "... " << std::endl;

					//if (modulo_frame == 0)
					//{
					
					//LoadPCA_Matrix(&pca_enc, &sdf, section, pca_chunk_dims, block_size);
					LoadPCA_MatrixInBlocks(&pca_enc, &sdf, section, pca_chunk_dims, block_size);
					double center = pca_enc.GetCenterOfData();
					pca_enc.AddValueToData(-center);
					//Eigen::VectorXd avg = pca_enc.GetRowAverages();
					//pca_enc.AddValuesToRows(-avg);
					//pca_enc.TransposeMatrix();
					pca_enc.ConductSVD();

					values[section_mux] = pca_enc.GetValues();

					double tot = values[section_mux].sum() * significance_thresh;
					size_t significant = FindSignificantValueCount(values[section_mux], significance_thresh, significance_epsilon);
					
					std::cout << "\t";

					for (size_t j = 0; j < values[section_mux].size(); ++j){
						if (values[section_mux][j] > 0){
							std::cout << values[section_mux][j] << "\t";
						}
					}

					if (significant == 0)
					{
						++zero_chunks;
					}

					std::cout << "\n\tSignificant Values (Top " << (significance_thresh * 100) << "%): " << significant << std::endl;

					total_pca_size += (significant * (1 + pca_enc.GetMat()->rows() + pca_enc.GetMat()->cols()) + 1) * sizeof(double);

					significant_values_this_frame += significant;

					if (significant > 0 && u_mats[section_mux].rows() > 0 && v_mats[section_mux].cols() > 0)
					{
						//Eigen::MatrixXd new_U = pca_enc.GetMatrixU().block(0, 0, u_mats[section_mux].rows(), significant);
						Eigen::MatrixXd new_U = pca_enc.GetMatrixU();
						//Eigen::MatrixXd new_V = pca_enc.GetMatrixV().transpose().block(0, 0, significant, v_mats[section_mux].cols());
						Eigen::MatrixXd new_V = pca_enc.GetMatrixV().transpose();

						//Eigen::MatrixXd old_U = u_mats[section_mux].block(0, 0, u_mats[section_mux].rows(), significant);
						Eigen::MatrixXd old_U = u_mats[section_mux];
						//Eigen::MatrixXd old_V = v_mats[section_mux].block(0, 0, significant, v_mats[section_mux].cols());
						Eigen::MatrixXd old_V = v_mats[section_mux];

						//double sim_thresh = 1;
						//size_t similar_bases = 0;
						//
						//double best_match_overall = DBL_MAX;
						//
						////Eigen::MatrixXd U_dots = new_U * old_U.transpose();
						//
						//for (size_t c = 0; c < old_U.cols(); ++c)
						//{
						//	bool has_best = false;
						//	for (size_t c2 = 0; c2 < new_U.cols(); ++c2)
						//	{
						//		double best_match = (new_U.col(c) - old_U.col(c2)).norm() * values[section_mux][c];
						//		best_match_overall = std::min(best_match_overall, best_match);
						//		if (best_match < sim_thresh && !has_best)
						//		{
						//			has_best = true;
						//			++similar_bases;
						//		}
						//	}
						//}
						//
						////std::cout << "Similar Bases (> " << sim_thresh << " dot): " << similar_bases << ", Most similar: " << best_match_overall << std::endl;
						//std::cout << "Similar Bases (< " << sim_thresh << " dist): " << similar_bases << ", Most similar: " << best_match_overall << std::endl;

						//Eigen::MatrixXd old_U_to_new_U = new_U * old_U.inverse();

						//std::cout << old_U_to_new_U.block(0, 0, 8, 8) << std::endl;
					}

					u_mats[section_mux] = pca_enc.GetMatrixU();
					v_mats[section_mux] = pca_enc.GetMatrixV().transpose();

					u_inv_mats[section_mux] = u_mats[section_mux].inverse();
					v_inv_mats[section_mux] = v_mats[section_mux].inverse();

					pca_enc.RecreateWithoutTransposingV(u_mats[section_mux], values[section_mux], v_mats[section_mux]);
					//pca_enc.TransposeMatrix();
					//pca_enc.AddValuesToRows(avg);
					pca_enc.AddValueToData(center);
					//EmplaceMatrix(*pca_enc.GetMat(), &sdf, section, pca_chunk_dims, block_size);
					EmplacePCA_MatrixInBlocks(&pca_enc, &sdf, section, pca_chunk_dims, block_size);
					//}
					//else
					//{
					//	LoadMatrix(temp_mat, &sdf, section, pca_chunk_dims, block_size);
					//
					//	emplacing_mat = u_inv_mats[section_mux] * temp_mat * v_inv_mats[section_mux];
					//
					//	Eigen::VectorXd diag = emplacing_mat.diagonal();
					//
					//	emplacing_mat = u_mats[section_mux] * diag.asDiagonal().toDenseMatrix() * v_mats[section_mux];
					//
					//	//std::cout << "Diagonal Fit: " << sqrt((diag - values[section_mux]).squaredNorm()) << std::endl;
					//
					//	EmplaceMatrix(emplacing_mat, &sdf, section, pca_chunk_dims, block_size);
					//}
				}
			}
		}

		//if (modulo_frame == 0)
		//{
		//	memcpy(iframe.data(), sdf.GetQuantizedGridPointer(), iframe.size());
		//}
		//else
		//{
		//	sdf.AddToGrid(iframe.data());
		//}

		std::cout << "Saving..." << std::endl;

		//sdf.QuantizeGrid();
		//sdf.ApplySigns(sign_data);
		auto to_save = sdf.ExtractMeshQuantized();// Eigen::Vector2i(1024, 1024), 0.1);
		to_save->ConcatenateMesh(*original_mesh, Eigen::Vector3d(1.0, 0, 0));
		to_save->ConcatenateMesh(input_mesh, Eigen::Vector3d(2.0, 0, 0));

		to_save->WriteOBJ(save_name);

		std::cout << "Significant values this frame: " << significant_values_this_frame << std::endl;

		total_significant_values += significant_values_this_frame;

		//std::cout << "Total significant values so far: " << total_significant_values << " (+ " << significant_values_this_frame << ")" << std::endl;
	}

	std::cout << "Total significant values required: " << total_significant_values << ", zero chunk count: " << zero_chunks << std::endl;

	std::cout << "Total original size: " << original_size << std::endl;

	std::cout << "Total LZ size: " << lz_size << std::endl;

	std::cout << "Total PCA Size: " << total_pca_size << std::endl;
}

size_t TemporalPCA_Suite::FindSignificantValueCount(Eigen::VectorXd& values, double significance_ratio, double epsilon)
{
	size_t sig_count = 0;
	double tot = significance_ratio * values.sum();

	for (size_t j = 0; j < values.size(); ++j)
	{
		if (tot <= epsilon || j >= significance_value_limit)
		{
			values[j] = 0;
		}
		else
		{
			++sig_count;
		}

		tot -= values[j];
	}

	return sig_count;
}

void TemporalPCA_Suite::run(int argc, char** argv)
{
	LoadSequences();
	InitializeSDF();
	InitializePCA_Encoder();

	//CheckAdjacentFrames();
	SaveAdjacentFramesWithPCA_Reuse(max_frames_to_process);
}
