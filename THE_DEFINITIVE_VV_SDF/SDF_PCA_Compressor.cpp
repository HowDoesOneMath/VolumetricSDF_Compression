#include "SDF_PCA_Compressor.h"

size_t SDF_PCA_Compressor::GetMultiplexedPartition(size_t x, size_t y, size_t z, size_t t)
{
    return pcad.partition_span_t * t + pcad.partition_span_x * x + pcad.partition_span_y * y + z;
}

void SDF_PCA_Compressor::ExtractSpatialBlockFromPCA(size_t file_location, size_t target_frame, size_t start_x, size_t start_y, size_t start_z)
{
    size_t sig_value_count;
    double avg;

    save_buffer.SetReaderLocation(file_location);

    save_buffer.ReadObjectFromBuffer(sig_value_count);
    save_buffer.ReadObjectFromBuffer(avg);

    if (sig_value_count <= 0)
    {
        for (size_t x = 0, sdf_coords_x = start_x * pcad.block_size_x; x < pcad.pca_reach_x; ++x, sdf_coords_x += pcad.block_size_x)
        {
            for (size_t y = 0, sdf_coords_y = start_y * pcad.block_size_y; y < pcad.pca_reach_y; ++y, sdf_coords_y += pcad.block_size_y)
            {
                for (size_t z = 0, sdf_coords_z = start_z * pcad.block_size_z; z < pcad.pca_reach_z; ++z, sdf_coords_z += pcad.block_size_z)
                {
                    sdf.SetQuantizedBlock(sdf_coords_x, sdf_coords_y, sdf_coords_z, pcad.block_size_x, pcad.block_size_y, pcad.block_size_z, avg + 0.1);
                }
            }
        }

        return;
    }

    size_t u_mat_rows;
    size_t v_mat_cols;

    save_buffer.ReadObjectFromBuffer(u_mat_rows);
    save_buffer.ReadObjectFromBuffer(v_mat_cols);

    size_t pca_row;

    Eigen::MatrixXd u_mat;
    Eigen::MatrixX<unsigned char> trunc_u_mat;

    Eigen::VectorXd d_values;
    Eigen::MatrixXd d_mat;

    Eigen::MatrixXd v_mat;
    Eigen::MatrixX<unsigned char> trunc_v_mat;

    Eigen::MatrixXd combined_mat;

    //u_mat.setZero(u_mat_rows, sig_value_count);
    u_mat.setZero(1, sig_value_count);
    d_values.setZero(sig_value_count);
    v_mat.setZero(sig_value_count, v_mat_cols);

    save_buffer.ReadVectorFromBuffer(d_values, 0, sig_value_count);

    double u_mat_max_val;
    double u_mat_min_val;

    double v_mat_max_val;
    double v_mat_min_val;

    save_buffer.ReadObjectFromBuffer(u_mat_max_val);
    save_buffer.ReadObjectFromBuffer(u_mat_min_val);

    size_t u_mat_location = save_buffer.GetReaderLocation();
    //save_buffer.SetReaderLocation(u_mat_location + target_frame * sig_value_count * sizeof(double));
    //save_buffer.ReadMatrixFromBuffer(u_mat, 0, 0, 1, sig_value_count);

    trunc_u_mat.setZero(1, sig_value_count);
    save_buffer.SetReaderLocation(u_mat_location + target_frame * sig_value_count * sizeof(unsigned char));
    //save_buffer.ReadMatrixFromBuffer(u_mat, 0, 0, 1, sig_value_count);
    save_buffer.ReadMatrixFromBuffer(trunc_u_mat, 0, 0, 1, sig_value_count);
    u_mat = UntruncateMatrix<unsigned char>(trunc_u_mat, u_mat_max_val, u_mat_min_val, matrix_truncation_maximum, 0);
    
    //std::vector<size_t> lz_u_locations(u_mat_rows + 1);
    //save_buffer.ReadArrayFromBuffer(lz_u_locations.data(), lz_u_locations.size());
    //save_buffer.SetReaderLocation(lz_u_locations[target_frame]);
    //std::vector<double> lz_u_data(sig_value_count);
    //lz_enc.ReadFromBuffer(save_buffer, reinterpret_cast<unsigned char*>(lz_u_data.data()), lz_u_data.size() * sizeof(double));
    //for (size_t c = 0; c < sig_value_count; ++c)
    //{
    //    u_mat(0, c) = lz_u_data[c];
    //}

    //size_t v_mat_location = u_mat_location + u_mat_rows * sig_value_count * sizeof(double);
    size_t v_mat_location = u_mat_location + u_mat_rows * sig_value_count * sizeof(unsigned char);
    //size_t v_mat_location = u_mat_location + u_mat_rows * sig_value_count * sizeof(unsigned char);
    //size_t v_mat_location = lz_u_locations[u_mat_rows];
    save_buffer.SetReaderLocation(v_mat_location);
    //save_buffer.ReadMatrixFromBuffer(v_mat, 0, 0, sig_value_count, v_mat_cols);

    save_buffer.ReadObjectFromBuffer(v_mat_max_val);
    save_buffer.ReadObjectFromBuffer(v_mat_min_val);

    trunc_v_mat.setZero(sig_value_count, v_mat_cols);
    save_buffer.ReadMatrixFromBuffer(trunc_v_mat, 0, 0, sig_value_count, v_mat_cols);
    v_mat = UntruncateMatrix<unsigned char>(trunc_v_mat, v_mat_max_val, v_mat_min_val, matrix_truncation_maximum, 0);

    //std::vector<double> lz_compressed_data(sig_value_count * v_mat_cols);
    //lz_enc.ReadFromBuffer(save_buffer, reinterpret_cast<unsigned char*>(lz_compressed_data.data()), lz_compressed_data.size() * sizeof(double));
    //
    //size_t lz_ind = 0;
    //for (size_t r = 0; r < sig_value_count; ++r)
    //{
    //    for (size_t c = 0; c < v_mat_cols; ++c, ++lz_ind)
    //    {
    //        v_mat(r, c) = lz_compressed_data[lz_ind];
    //    }
    //}

    d_mat = d_values.asDiagonal();

    Eigen::MatrixXd sdf_block;

    std::vector<double> pca_data_temp;

    for (size_t x = 0, sdf_coords_x = start_x * pcad.block_size_x; x < pcad.pca_reach_x; ++x, sdf_coords_x += pcad.block_size_x)
    {
        for (size_t y = 0, sdf_coords_y = start_y * pcad.block_size_y; y < pcad.pca_reach_y; ++y, sdf_coords_y += pcad.block_size_y)
        {
            for (size_t z = 0, sdf_coords_z = start_z * pcad.block_size_z; z < pcad.pca_reach_z; ++z, sdf_coords_z += pcad.block_size_z)
            {
                //pca_row = target_frame * pcad.pca_reach_span_t + x * pcad.pca_reach_span_x + y * pcad.pca_reach_span_y + z; 
                //sdf_block = u_mat.block(pca_row, 0, 1, sig_value_count) * d_mat * v_mat;
                sdf_block = (u_mat * d_mat) * v_mat;

                sdf.InsertBlock(sdf_coords_x, sdf_coords_y, sdf_coords_z, pcad.block_size_x, pcad.block_size_y, pcad.block_size_z, sdf_block, avg);
            }
        }
    }
}

void SDF_PCA_Compressor::CarryOverRLE_InRange(size_t start_t, size_t end_t,
    size_t start_x, size_t start_y, size_t start_z, std::vector<size_t>& rle_file_locations)
{
    size_t end_x = start_x + pcad.pca_reach_x;
    size_t end_y = start_y + pcad.pca_reach_y;
    size_t end_z = start_z + pcad.pca_reach_z;

    size_t rle_array_loc;

    size_t current_length;
    size_t actual_length;

    size_t lengths_sum = 0;
    size_t total_runs = 0;

    size_t is_unique;

    for (size_t t = start_t; t < end_t; ++t)
    {
        for (size_t x = start_x; x < end_x; ++x)
        {
            for (size_t y = start_y; y < end_y; ++y)
            {
                for (size_t z = start_z; z < end_z; ++z)
                {
                    rle_array_loc = GetMultiplexedPartition(x, y, z, t);

                    save_buffer.SetReaderLocation(rle_file_locations[rle_array_loc]);

                    while (lengths_sum < pcad.total_block_size)
                    {
                        save_buffer.ReadObjectFromBuffer(current_length);
                        save_buffer.WriteObjectToBuffer(current_length);

                        actual_length = current_length & rle_enc.remove_unique_and_negative;
                        lengths_sum += actual_length;

                        is_unique = (current_length & rle_enc.signal_unique);
                        total_runs += (is_unique > 0) * actual_length + (is_unique == 0);
                    }

                    rle_enc.runs.resize(total_runs);
                    save_buffer.ReadArrayFromBuffer(rle_enc.runs.data(), rle_enc.runs.size());
                    save_buffer.WriteArrayToBuffer(rle_enc.runs.data(), rle_enc.runs.size());
                }
            }
        }
    }
}

size_t SDF_PCA_Compressor::SignificantValueCountInPCA(PCA_Dimensions& pca_dim)
{
    Eigen::VectorXd sigma_values = pca_enc.GetValues();

    size_t max_amount = std::min(pca_dim.max_allowed_components, (size_t)sigma_values.size());
    size_t sig_count = 0;

    double max_allowed = pca_dim.significant_value_ratio * sigma_values.sum();

    for (; sig_count < sigma_values.size(); ++sig_count)
    {
        if (max_allowed <= 0 || sig_count >= max_amount)
        {
            break;
        }
        
        max_allowed -= sigma_values[sig_count];

        //if (sigma_values[sig_count] < pca_dim.significant_value_threshold)
        //{
        //    break;
        //}
    }

    return sig_count;
}

bool SDF_PCA_Compressor::ParseFlag(size_t& to_parse, size_t flag)
{
    bool exists = to_parse & flag;
    to_parse &= ~flag;
    return exists;
}

std::shared_ptr<VV_Mesh> SDF_PCA_Compressor::ExtractSingleMeshFromIntermediary(std::vector<size_t>& file_locations, size_t mesh_index)
{
    size_t starting_loc = file_locations[mesh_index * pcad.total_partitions];

    //std::cout << starting_loc << std::endl;

    save_buffer.SetReaderLocation(starting_loc);

    std::vector<unsigned char> block_buffer;
    block_buffer.resize(pcad.total_block_size);

    for (size_t x = 0; x < sdf.GetDimX(); x += pcad.block_size_x)
    {
        for (size_t y = 0; y < sdf.GetDimY(); y += pcad.block_size_y)
        {
            for (size_t z = 0; z < sdf.GetDimZ(); z += pcad.block_size_z)
            {
                lz_enc.ReadFromBuffer(save_buffer, block_buffer.data(), block_buffer.size());
                sdf.InsertBlock(x, y, z, pcad.block_size_x, pcad.block_size_y, pcad.block_size_z, block_buffer.data());
            }
        }
    }

    return sdf.ExtractMeshQuantized();// Eigen::Vector2i(1024, 1024), 0.126);
}

size_t SDF_PCA_Compressor::MarshallRLE_toPCA(size_t start_t, size_t end_t, size_t start_x, size_t start_y, size_t start_z,
    std::vector<size_t>& rle_file_locations, std::vector<unsigned char>& block_buffer)
{
    size_t end_x = start_x + pcad.pca_reach_x;
    size_t end_y = start_y + pcad.pca_reach_y;
    size_t end_z = start_z + pcad.pca_reach_z;

    size_t rle_array_loc;

    size_t lengths_sum;
    size_t current_length;

    size_t pca_row = 0;

    size_t rle_size = 0;

    //Eigen::VectorXd avgs;
    //avgs.resize(std::min(pca_dim.total_pca_reach, pca_dim.frame_count * pca_dim.pca_reach_per_frame));

    for (size_t t = start_t; t < end_t; ++t)
    {
        for (size_t x = start_x; x < end_x; ++x)
        {
            for (size_t y = start_y; y < end_y; ++y)
            {
                for (size_t z = start_z; z < end_z; ++z, ++pca_row)
                {
                    rle_array_loc = GetMultiplexedPartition(x, y, z, t);

                    save_buffer.SetReaderLocation(rle_file_locations[rle_array_loc]);

                    rle_enc.LoadFromBuffer(save_buffer, pcad.total_block_size);

                    rle_size += (rle_enc.lengths.size() * sizeof(size_t) + rle_enc.runs.size());

                    rle_enc.ExtractOriginal(block_buffer.data());

                    pca_enc.AddData(pca_row, block_buffer.data());
                }
            }
        }
    }

    //std::cout << avgs.transpose() << std::endl;

    return rle_size;
}

size_t SDF_PCA_Compressor::MarshallLZ_toPCA(size_t start_t, size_t end_t, size_t start_x, size_t start_y, size_t start_z, std::vector<size_t>& lz_file_locations, std::vector<unsigned char>& block_buffer)
{
    size_t end_x = start_x + pcad.pca_reach_x;
    size_t end_y = start_y + pcad.pca_reach_y;
    size_t end_z = start_z + pcad.pca_reach_z;

    size_t lz_array_loc;

    size_t pca_row = 0;

    size_t lz_size = 0;

    for (size_t t = start_t; t < end_t; ++t)
    {
        for (size_t x = start_x; x < end_x; ++x)
        {
            for (size_t y = start_y; y < end_y; ++y)
            {
                for (size_t z = start_z; z < end_z; ++z, ++pca_row)
                {
                    lz_array_loc = GetMultiplexedPartition(x, y, z, t);

                    save_buffer.SetReaderLocation(lz_file_locations[lz_array_loc]);

                    lz_size += lz_enc.ReadFromBuffer(save_buffer, block_buffer.data(), block_buffer.size());

                    pca_enc.AddData(pca_row, block_buffer.data());
                }
            }
        }
    }

    return lz_size;
}

std::shared_ptr<VV_Mesh> SDF_PCA_Compressor::ExtractSingleMesh(std::vector<size_t>& block_locations, std::vector<size_t>& sign_locations, size_t mesh_index_in_SVD)
{
    sdf.RefreshGrid();

    size_t partition_number = mesh_index_in_SVD / pcad.pca_reach_t;
    size_t place_in_partition = mesh_index_in_SVD % pcad.pca_reach_t;

    size_t exact_array_start = partition_number * pcad.total_pca_reach;

    size_t array_loc;

    Eigen::MatrixXd extracted_u;
    Eigen::VectorXd extracted_s;
    Eigen::MatrixXd extracted_v;

    for (size_t x = 0, array_x = 0; x < pcad.partitions_x; x += pcad.pca_reach_x, ++array_x)
    {
        for (size_t y = 0, array_y = 0; y < pcad.partitions_y; y += pcad.pca_reach_y, ++array_y)
        {
            for (size_t z = 0, array_z = 0; z < pcad.partitions_z; z += pcad.pca_reach_z, ++array_z)
            {
                array_loc = partition_number * pcad.pca_span_t + array_x * pcad.pca_span_x + array_y * pcad.pca_span_y + array_z;

                ExtractSpatialBlockFromPCA(block_locations[array_loc], place_in_partition, x, y, z);
            }
        }
    }

    //sdf.PrintCrossSectionOfQuantizedGrid(60);

    std::vector<unsigned char> compressed_signs;
    save_buffer.SetReaderLocation(sign_locations[mesh_index_in_SVD]);
    //std::cout << "Reading signs from: " << sign_locations[mesh_index_in_SVD] << std::endl;
    save_buffer.ReadVectorFromBuffer(compressed_signs);
    //std::cout << "Vector size: " << compressed_signs.size() << std::endl;

    std::vector<unsigned char> actual_signs(sdf.GetQuantizedGridLength() / 8);
    lz_enc.DecompressData(compressed_signs.data(), compressed_signs.size(), actual_signs);
    //sdf.ApplySigns(actual_signs);

    auto to_return = sdf.ExtractMeshQuantized();// Eigen::Vector2i(1024, 1024), 0.126);

    return to_return;
}

bool SDF_PCA_Compressor::FillPCA_Struct(GridDataStruct& gds, Eigen::Vector3i block_size, Eigen::Vector4i pca_reach, 
    double significant_value_ratio, size_t max_allowed_components)
{
    pcad.gds = gds;

    pcad.block_size_x = block_size.x();
    pcad.block_size_y = block_size.y();
    pcad.block_size_z = block_size.z();

    if ((gds.dim_x % pcad.block_size_x != 0) || (gds.dim_y % pcad.block_size_y != 0) || (gds.dim_z % pcad.block_size_z != 0))
    {
        std::cout << "INVALID GRID DIMENSIONS AND/OR BLOCK DIMENSIONS:\tgrid: " <<
            gds.dim_x << ", " << gds.dim_y << ", " << gds.dim_z << "\t\tblock: " <<
            block_size.x() << ", " << block_size.y() << ", " << block_size.z() << std::endl;
        return false;
    }

    pcad.partitions_x = gds.dim_x / block_size.x();
    pcad.partitions_y = gds.dim_y / block_size.y();
    pcad.partitions_z = gds.dim_z / block_size.z();

    pcad.pca_reach_x = pca_reach.x();
    pcad.pca_reach_y = pca_reach.y();
    pcad.pca_reach_z = pca_reach.z();
    pcad.pca_reach_t = pca_reach.w();

    if ((pcad.partitions_x % pcad.pca_reach_x != 0) || (pcad.partitions_y % pcad.pca_reach_y != 0) || (pcad.partitions_z % pcad.pca_reach_z != 0))
    {
        std::cout << "INVALID PCA X/Y/Z REACH:\tpartitions: " <<
            pcad.partitions_x << ", " << pcad.partitions_y << ", " << pcad.partitions_z << "\t\treach: " <<
            pcad.pca_reach_x << ", " << pcad.pca_reach_y << ", " << pcad.pca_reach_z << std::endl;
        return false;
    }

    pcad.max_allowed_components = max_allowed_components;

    pcad.significant_value_ratio = significant_value_ratio;

    pcad.CalculateHelperValues();

    return true;
}

void SDF_PCA_Compressor::PrintSingularValues(Eigen::VectorXd &values, size_t value_limit)
{
    size_t min_limit = std::min(value_limit, (size_t)values.size());

    for (size_t i = 0; i < min_limit; ++i)
    {
        std::cout << values[i] << "\t";
    }

    std::cout << std::endl;
}

void SDF_PCA_Compressor::PrintSingularValuesAndCoords(size_t x, size_t y, size_t z, size_t t, Eigen::VectorXd& values, size_t value_limit)
{
    size_t min_limit = std::min(value_limit, (size_t)values.size());

    std::cout << "Singular Values At (t --- x,y,z) " << t << " --- " << x << ", " << y << ", " << z << " (num: " << min_limit << "): " << std::endl;

    for (size_t i = 0; i < min_limit; ++i)
    {
        std::cout << values[i] << "\t";
    }

    std::cout << std::endl;
}

bool SDF_PCA_Compressor::PreInitialization(GridDataStruct& gds, Eigen::Vector3i block_size, Eigen::Vector4i pca_reach, double significant_value_ratio, size_t max_allowed_components)
{
    if (!FillPCA_Struct(gds, block_size, pca_reach, significant_value_ratio, max_allowed_components))
    {
        std::cout << "PROBLEMS FILLING PCA STRUCT" << std::endl;
        return false;
    }

    sdf.ClearGrid();
    if (!sdf.InitializeGrid(pcad.gds))
    {
        std::cout << "COULDN'T INITIALIZE GRID!" << std::endl;
        return false;
    }

    return true;
}

bool SDF_PCA_Compressor::SaveIntermediaryFile(std::string input_folder, std::string intermediary_file_name,
    double shell_buffer, double mesh_maximum_artifact_size)
{
    if (!sf.FindFiles(input_folder, mesh_sf))
    {
        std::cout << "Couldn't open input folder to read!" << std::endl;
        return false;
    }

    if (!save_buffer.OpenWriteBuffer(intermediary_file_name))
    {
        std::cout << "Couldn't open intermediary file to write!" << std::endl;
        return false;
    }

    std::vector<size_t> file_locations_blocks;
    file_locations_blocks.resize(pcad.total_partitions * sf.files[mesh_sf.key].size());
    std::vector<size_t> file_locations_signs;
    file_locations_signs.resize(sf.files[mesh_sf.key].size());

    size_t current_block_location = 0;

    std::vector<unsigned char> block_buffer;
    block_buffer.resize(pcad.total_block_size);

    std::cout << "BLOCK BUFFER SIZE: " << block_buffer.size() << std::endl;

    pcad.frame_count = sf.files[mesh_sf.key].size();

    pcad.WriteToBuffer(save_buffer);

    pcad.DebugStats();

    size_t file_locations_blocks_start = save_buffer.GetWriterLocation();
    save_buffer.WriteArrayToBuffer(file_locations_blocks.data(), file_locations_blocks.size());

    size_t file_locations_signs_start = save_buffer.GetWriterLocation();
    save_buffer.WriteArrayToBuffer(file_locations_signs.data(), file_locations_signs.size());
    
    std::vector<unsigned char> signs;
    std::vector<unsigned char> compressed_signs;
    size_t total_signs_size = 0;

    for (int i = 0; i < sf.files[mesh_sf.key].size(); ++i)
    {
        //std::cout << "RLE of frame " << i << "... " << std::endl;
        std::cout << "LZ of frame " << i << "... " << std::endl;

        std::cout << save_buffer.GetWriterLocation() << std::endl;

        if (!mesh.ReadOBJ(sf.files[mesh_sf.key][i]))
        {
            std::cout << "ERROR: mesh could not be loaded: " << sf.files[mesh_sf.key][i] << std::endl;
            return false;
        }

        mp.CreateUnionFindPartitions(mesh);
        mp.NegateInsignificantPartitions(mesh, mesh_maximum_artifact_size);
        mesh.ClearNegativeTriangles();

        if (!sdf.CastMeshUnsignedDistance(&mesh, shell_buffer))
        {
            std::cout << "ERROR: mesh could not be cast into SDF: " << sf.files[mesh_sf.key][i] << std::endl;
            return false;
        }

        for (size_t x = 0; x < sdf.GetDimX(); x += pcad.block_size_x)
        {
            for (size_t y = 0; y < sdf.GetDimY(); y += pcad.block_size_y)
            {
                for (size_t z = 0; z < sdf.GetDimZ(); z += pcad.block_size_z)
                {
                    sdf.ExtractBlock(x, y, z, pcad.block_size_x, pcad.block_size_y, pcad.block_size_z, block_buffer.data());
                    //sdf.ExtractBlockMortonOrder(x, y, z, block_buffer.size(), block_buffer.data());

                    file_locations_blocks[current_block_location] = save_buffer.GetWriterLocation();
                    ++current_block_location;

                    lz_enc.SaveToBuffer(save_buffer, block_buffer.data(), block_buffer.size());

                    //rle_enc.GetRunLength(block_buffer.data(), block_buffer.size());

                    //rle_enc.SaveToBuffer(save_buffer);
                }
            }
        }

        file_locations_signs[i] = save_buffer.GetWriterLocation();

        sdf.HarvestSigns(signs);
        lz_enc.CompressData(signs.data(), signs.size(), compressed_signs);
        save_buffer.WriteVectorToBuffer(compressed_signs);
        total_signs_size += compressed_signs.size();

        std::cout << "Signs size: " << compressed_signs.size() << std::endl;
    }

    std::cout << "Total signs size: " << total_signs_size << std::endl;

    size_t writer_eof = save_buffer.GetWriterLocation();

    save_buffer.SetWriterLocation(file_locations_blocks_start);
    save_buffer.WriteArrayToBuffer(file_locations_blocks.data(), file_locations_blocks.size());

    save_buffer.SetWriterLocation(file_locations_signs_start);
    save_buffer.WriteArrayToBuffer(file_locations_signs.data(), file_locations_signs.size());

    save_buffer.SetWriterLocation(writer_eof);
    save_buffer.CloseWriteBuffer();

    return true;
}

bool SDF_PCA_Compressor::SavePCA_File(std::string intermediary_file_name, std::string pca_file_name)
{
    if (!save_buffer.OpenReadBuffer(intermediary_file_name))
    {
        std::cout << "Couldn't read intermediary file storage!" << std::endl;
        return false;
    }

    if (!save_buffer.OpenWriteBuffer(pca_file_name))
    {
        std::cout << "Couldn't open PCA file to write!" << std::endl;
        return false;
    }

    pcad.ReadFromBuffer(save_buffer);
    pcad.WriteToBuffer(save_buffer);

    sdf.ClearGrid();
    sdf.InitializeGrid(pcad.gds);

    std::vector<size_t> segment_locations(pcad.total_partitions * pcad.frame_count);
    save_buffer.ReadArrayFromBuffer(segment_locations.data(), pcad.total_partitions * pcad.frame_count);

    std::vector<size_t> sign_locations(pcad.frame_count);
    save_buffer.ReadArrayFromBuffer(sign_locations.data(), pcad.frame_count);

    pca_enc.Initialize(std::min(pcad.total_pca_reach, pcad.frame_count * pcad.pca_reach_per_frame), pcad.total_block_size);

    size_t target_frame;

    std::vector<unsigned char> block_buffer;
    block_buffer.resize(pcad.total_block_size);

    size_t significant_values = 0;

    size_t generic_total_size;
    size_t pca_total_size;

    size_t u_mat_rows;
    size_t v_mat_cols;
    Eigen::VectorXd sig_vals;
    Eigen::MatrixXd u_mat;
    Eigen::MatrixXd v_mat;

    size_t current_file_location = 0;


    size_t total_t_partitions = pcad.frame_count / pcad.pca_reach_t + ((pcad.frame_count % pcad.pca_reach_t) != 0);
    std::vector<size_t> svd_block_locations(total_t_partitions * (pcad.total_partitions / pcad.pca_reach_per_frame));
    size_t svd_blocks_size = svd_block_locations.size();
    save_buffer.WriteObjectToBuffer(svd_blocks_size);
    size_t svd_block_locations_start = save_buffer.GetWriterLocation();
    save_buffer.WriteArrayToBuffer(svd_block_locations.data(), svd_block_locations.size());

    size_t signs_start = save_buffer.GetWriterLocation();
    save_buffer.WriteArrayToBuffer(sign_locations.data(), sign_locations.size());

    std::vector<unsigned char> sign_array;

    std::cout << "Saving signs..." << std::endl;

    size_t total_sign_size = 0;

    for (size_t i = 0; i < sign_locations.size(); ++i)
    {
        std::cout << "Input Sign Location: " << sign_locations[i] << "\t";

        save_buffer.SetReaderLocation(sign_locations[i]);
        save_buffer.ReadVectorFromBuffer(sign_array);
        sign_locations[i] = save_buffer.GetWriterLocation();
        save_buffer.WriteVectorToBuffer(sign_array);

        std::cout << "Output Sign Location: " << sign_locations[i] << std::endl;

        total_sign_size += sign_array.size();
    }

    std::cout << "Total signs size: " << total_sign_size << std::endl;

    size_t blocks_start = save_buffer.GetWriterLocation();

    save_buffer.SetWriterLocation(signs_start);
    save_buffer.WriteArrayToBuffer(sign_locations.data(), sign_locations.size());
    save_buffer.SetWriterLocation(blocks_start);

    auto pca_start = std::chrono::high_resolution_clock::now();

    for (size_t t = 0; t < pcad.frame_count; t += pcad.pca_reach_t)
    {
        target_frame = std::min(t + pcad.pca_reach_t, pcad.frame_count);

        std::cout << "PCA of frames " << t << " to " << (target_frame - 1) << "... " << std::endl;

        for (size_t x = 0; x < pcad.partitions_x; x += pcad.pca_reach_x)
        {
            for (size_t y = 0; y < pcad.partitions_y; y += pcad.pca_reach_y)
            {
                for (size_t z = 0; z < pcad.partitions_z; z += pcad.pca_reach_z, ++current_file_location)
                {
                    //rle_total_size = MarshallRLE_toPCA(t, target_frame, x, y, z, rle_locations, block_buffer);
                    generic_total_size = MarshallLZ_toPCA(t, target_frame, x, y, z, segment_locations, block_buffer);

                    //std::cout << "PCAing coords (" << t << "): " << x << ", " << y << ", " << z << "..." << std::endl;

                    double mat_avg = pca_enc.GetCenterOfData();
                    pca_enc.AddValueToData(-mat_avg);

                    pca_enc.ConductSVD();

                    significant_values = SignificantValueCountInPCA(pcad);

                    sig_vals = pca_enc.GetValues();

                    u_mat = pca_enc.GetMatrixU();
                    u_mat_rows = u_mat.rows();
                    v_mat = pca_enc.GetMatrixV().transpose();
                    v_mat_cols = v_mat.cols();

                    svd_block_locations[current_file_location] = save_buffer.GetWriterLocation();

                    pca_total_size = svd_block_locations[current_file_location];

                    save_buffer.WriteObjectToBuffer(significant_values);
                    save_buffer.WriteObjectToBuffer(mat_avg);

                    if (significant_values > 0)
                    {
                        std::vector<double> to_lz_compress;
                        //std::vector<size_t> lz_u_indices;

                        save_buffer.WriteObjectToBuffer(u_mat_rows);
                        save_buffer.WriteObjectToBuffer(v_mat_cols);

                        save_buffer.WriteVectorToBuffer(sig_vals, 0, significant_values);

                        Eigen::MatrixXd sub_u_mat = u_mat.block(0, 0, u_mat_rows, significant_values);
                        double u_mat_max_val = sub_u_mat.maxCoeff();
                        double u_mat_min_val = sub_u_mat.minCoeff();
                        Eigen::MatrixX<unsigned char> trunc_u_mat = TruncateMatrix<unsigned char>(sub_u_mat, u_mat_max_val, u_mat_min_val, matrix_truncation_maximum, 0);

                        save_buffer.WriteObjectToBuffer(u_mat_max_val);
                        save_buffer.WriteObjectToBuffer(u_mat_min_val);
                        for (size_t r = 0; r < u_mat_rows; ++r)
                        {
                            save_buffer.WriteMatrixToBuffer(trunc_u_mat, r, 0, r + 1, significant_values);
                        }

                        Eigen::MatrixXd sub_v_mat = v_mat.block(0, 0, significant_values, v_mat_cols);
                        double v_mat_max_val = sub_v_mat.maxCoeff();
                        double v_mat_min_val = sub_v_mat.minCoeff();
                        Eigen::MatrixX<unsigned char> trunc_v_mat = TruncateMatrix<unsigned char>(sub_v_mat, v_mat_max_val, v_mat_min_val, matrix_truncation_maximum, 0);

                        //save_buffer.WriteMatrixToBuffer(v_mat, 0, 0, significant_values, v_mat_cols);
                        save_buffer.WriteObjectToBuffer(v_mat_max_val);
                        save_buffer.WriteObjectToBuffer(v_mat_min_val);
                        save_buffer.WriteMatrixToBuffer(trunc_v_mat, 0, 0, significant_values, v_mat_cols);
                    }

                    pca_total_size = (size_t)save_buffer.GetWriterLocation() - pca_total_size;
                }
            }
        }

        //It's only possible to get a half-filled matrix when switching from one frame group to another
        pca_enc.RefreshMatrix();
    }

    auto pca_end = std::chrono::high_resolution_clock::now();

    std::cout << "Time taken: " << ((pca_end - pca_start).count() * 0.000000001) << std::endl;

    size_t writer_eof = save_buffer.GetWriterLocation();

    save_buffer.SetWriterLocation(svd_block_locations_start);
    save_buffer.WriteArrayToBuffer(svd_block_locations.data(), svd_block_locations.size());

    save_buffer.SetWriterLocation(writer_eof);
    save_buffer.CloseWriteBuffer();
    save_buffer.CloseReadBuffer();

    return true;
}

bool SDF_PCA_Compressor::ReconstructMeshesFromPCA(std::string pca_file_name, std::string output_folder, std::string output_file_tag, int digit_count)
{
    if (!save_buffer.OpenReadBuffer(pca_file_name))
    {
        std::cout << "PROBLEMS OPENING PCA FILE" << std::endl;
        return false;
    }

    pcad.ReadFromBuffer(save_buffer);

    sdf.ClearGrid();
    if (!sdf.InitializeGrid(pcad.gds))
    {
        std::cout << "COULDN'T INITIALIZE GRID!" << std::endl;
        return false;
    }

    pca_enc.Initialize(std::min(pcad.total_pca_reach, pcad.frame_count * pcad.pca_reach_per_frame), pcad.total_block_size);

    std::vector<size_t> block_locations;
    save_buffer.ReadVectorFromBuffer(block_locations);
    std::vector<size_t> sign_locations(pcad.frame_count);
    save_buffer.ReadArrayFromBuffer(sign_locations.data(), pcad.frame_count);

    double decompression_time;
    double total_decompression_time = 0;

    for (size_t index = 0; index < pcad.frame_count; ++index)
    {   
        std::cout << "Extracting Mesh " << index << "..." << std::endl;

        auto time_begin = std::chrono::high_resolution_clock::now();

        auto new_mesh = ExtractSingleMesh(block_locations, sign_locations, index);

        auto time_delta = std::chrono::high_resolution_clock::now() - time_begin;

        decompression_time = time_delta.count() * 0.000000001;
        std::cout << "Elapsed (decompress): " << decompression_time << "s" << std::endl;
        total_decompression_time += decompression_time;
        
        std::string save_name = output_folder + "/" + output_file_tag + GetNumberFixedLength(index, digit_count) + ".obj";

        new_mesh->WriteOBJ(save_name);
    }

    std::cout << "Total decompression time: " << total_decompression_time << std::endl;

    return true;
}

bool SDF_PCA_Compressor::ReconstructMeshesFromIntermediary(std::string intermediary_file_name, std::string output_folder, std::string output_file_tag, int digit_count)
{
    if (!save_buffer.OpenReadBuffer(intermediary_file_name))
    {
        std::cout << "PROBLEMS OPENING INTERMEDIARY FILE" << std::endl;
        return false;
    }

    pcad.ReadFromBuffer(save_buffer);
    pcad.DebugStats();

    sdf.ClearGrid();
    if (!sdf.InitializeGrid(pcad.gds))
    {
        std::cout << "COULDN'T INITIALIZE GRID!" << std::endl;
        return false;
    }

    std::vector<size_t> file_locations(pcad.frame_count * pcad.total_partitions);
    save_buffer.ReadArrayFromBuffer(file_locations.data(), file_locations.size());

    for (size_t index = 0; index < pcad.frame_count; ++index)
    {
        std::cout << "Saving mesh " << index << "..." << std::endl;
        auto new_mesh = ExtractSingleMeshFromIntermediary(file_locations, index);

        std::string save_name = output_folder + "/" + output_file_tag + GetNumberFixedLength(index, digit_count) + ".obj";

        new_mesh->WriteOBJ(save_name);
    }

    save_buffer.CloseReadBuffer();
}

bool SDF_PCA_Compressor::TestSpecificSection(GridDataStruct& gds, std::string input_folder, Eigen::Vector4i target_location, Eigen::Vector3i block_size, 
    double shell_buffer, Eigen::Vector4i pca_reach, double significant_value_threshold, size_t max_allowed_components, double mesh_maximum_artifact_size)
{
    if (!PreInitialization(gds, block_size, pca_reach, significant_value_threshold, max_allowed_components))
    {
        return false;
    }

    if (!sf.FindFiles(input_folder, mesh_sf))
    {
        std::cout << "Couldn't open input folder to read!" << std::endl;
        return false;
    }

    std::vector<unsigned char> block_buffer;
    block_buffer.resize(pcad.total_block_size);

    size_t end_loc_t = std::min(sf.files[mesh_sf.key].size(), target_location.w() + pcad.pca_reach_t);
    size_t end_loc_x = (target_location.x() + pcad.pca_reach_x) * pcad.block_size_x;
    size_t end_loc_y = (target_location.y() + pcad.pca_reach_y) * pcad.block_size_y;
    size_t end_loc_z = (target_location.z() + pcad.pca_reach_z) * pcad.block_size_z;

    size_t pca_row = 0;

    pca_enc.Initialize(pcad.total_pca_reach, pcad.total_block_size);

    std::cout << "PCA SIZE: " << pca_enc.GetMat()->rows() << ", " << pca_enc.GetMat()->cols() << std::endl;

    for (size_t t = target_location.w(); t < end_loc_t; ++t)
    {
        std::cout << "On frame " << t << "... " << std::endl;

        if (!mesh.ReadOBJ(sf.files[mesh_sf.key][t]))
        {
            std::cout << "ERROR: mesh could not be loaded: " << sf.files[mesh_sf.key][t] << std::endl;
            return false;
        }

        mp.CreateUnionFindPartitions(mesh);
        mp.NegateInsignificantPartitions(mesh, mesh_maximum_artifact_size);
        mesh.ClearNegativeTriangles();

        if (!sdf.CastMeshUnsignedDistance(&mesh, shell_buffer))
        {
            std::cout << "ERROR: mesh could not be cast into SDF: " << sf.files[mesh_sf.key][t] << std::endl;
            return false;
        }


        for (size_t x = target_location.x() * pcad.block_size_x; x < end_loc_x; x += pcad.block_size_x)
        {
            for (size_t y = target_location.y() * pcad.block_size_y; y < end_loc_y; y += pcad.block_size_y)
            {
                for (size_t z = target_location.z() * pcad.block_size_z; z < end_loc_z; z += pcad.block_size_z, ++pca_row)
                {
                    sdf.ExtractBlock(x, y, z, pcad.block_size_x, pcad.block_size_y, pcad.block_size_z, block_buffer.data());

                    pca_enc.AddData(pca_row, block_buffer.data());
                }
            }
        }
    }

    std::cout << "PCA-ing..." << std::endl;

    Eigen::MatrixXd* pca_mat = pca_enc.GetMat();

    for (size_t r = 0; r < pca_mat->rows(); ++r)
    {
        double col_sum = 0;

        for (size_t c = 0; c < pca_mat->cols(); ++c)
        {
            col_sum += (*pca_mat)(r, c);
        }

        std::cout << "\t" << col_sum;
    }

    std::cout << std::endl;

    pca_enc.ConductSVD();

    std::cout << "Did it make it?" << std::endl;

    return true;
}

void SDF_PCA_Compressor::PCA_Dimensions::CalculateHelperValues()
{
    total_block_size = block_size_x * block_size_y * block_size_z;
    total_partitions = partitions_x * partitions_y * partitions_z;

    partition_span_y = partitions_z;
    partition_span_x = partitions_y * partition_span_y;
    partition_span_t = partitions_x * partition_span_x;

    pca_reach_per_frame = pca_reach_x * pca_reach_y * pca_reach_z;
    total_pca_reach = pca_reach_t * pca_reach_per_frame;

    single_frame_size = total_block_size * total_partitions;

    pca_sections_x = partitions_x / pca_reach_x;
    pca_sections_y = partitions_y / pca_reach_y;
    pca_sections_z = partitions_z / pca_reach_z;

    pca_span_y = pca_sections_z;
    pca_span_x = pca_sections_y * pca_span_y;
    pca_span_t = pca_sections_x * pca_span_x;

    pca_reach_span_y = pca_reach_z;
    pca_reach_span_x = pca_reach_span_y * pca_reach_y;
    pca_reach_span_t = pca_reach_span_x * pca_reach_x;
}

void SDF_PCA_Compressor::PCA_Dimensions::WriteToBuffer(VV_SaveFileBuffer& buffer)
{
    gds.WriteToBuffer(buffer);

    buffer.WriteObjectToBuffer(frame_count);

    buffer.WriteObjectToBuffer(block_size_x);
    buffer.WriteObjectToBuffer(block_size_y);
    buffer.WriteObjectToBuffer(block_size_z);

    buffer.WriteObjectToBuffer(partitions_x);
    buffer.WriteObjectToBuffer(partitions_y);
    buffer.WriteObjectToBuffer(partitions_z);

    buffer.WriteObjectToBuffer(pca_reach_x);
    buffer.WriteObjectToBuffer(pca_reach_y);
    buffer.WriteObjectToBuffer(pca_reach_z);
    buffer.WriteObjectToBuffer(pca_reach_t);

    buffer.WriteObjectToBuffer(max_allowed_components);

    buffer.WriteObjectToBuffer(significant_value_ratio);
}

void SDF_PCA_Compressor::PCA_Dimensions::ReadFromBuffer(VV_SaveFileBuffer& buffer)
{
    gds.ReadFromBuffer(buffer);

    buffer.ReadObjectFromBuffer(frame_count);

    buffer.ReadObjectFromBuffer(block_size_x);
    buffer.ReadObjectFromBuffer(block_size_y);
    buffer.ReadObjectFromBuffer(block_size_z);

    buffer.ReadObjectFromBuffer(partitions_x);
    buffer.ReadObjectFromBuffer(partitions_y);
    buffer.ReadObjectFromBuffer(partitions_z);

    buffer.ReadObjectFromBuffer(pca_reach_x);
    buffer.ReadObjectFromBuffer(pca_reach_y);
    buffer.ReadObjectFromBuffer(pca_reach_z);
    buffer.ReadObjectFromBuffer(pca_reach_t);

    buffer.ReadObjectFromBuffer(max_allowed_components);

    buffer.ReadObjectFromBuffer(significant_value_ratio);

    CalculateHelperValues();
}

void SDF_PCA_Compressor::PCA_Dimensions::DebugStats()
{
    std::cout << "PCA stats: " << std::endl;

    std::cout << "\tFrame count: " << frame_count << std::endl;
    std::cout << "\tFrame size: " << single_frame_size << std::endl;

    std::cout << "\tBlock size: " << block_size_x << ", " << block_size_y << ", " << block_size_z << " (" << total_block_size <<  ")" << std::endl;
    std::cout << "\tPartitions: " << partitions_x << ", " << partitions_y << ", " << partitions_z << " (" << total_partitions <<  ")" << std::endl;
    std::cout << "\tPartition Spans: " << partition_span_x << ", " << partition_span_y << ", " << partition_span_t << std::endl;

    std::cout << "\tPCA reaches: " << pca_reach_x << ", " << pca_reach_y << ", " << pca_reach_z << ", " << pca_reach_t << 
        " (" << pca_reach_per_frame << ", " << total_pca_reach << ")" << std::endl;

    std::cout << "\tPCA sections: " << pca_sections_x << ", " << pca_sections_z << ", " << pca_sections_z << std::endl;
    std::cout << "\tPCA spans: " << pca_span_x << ", " << pca_span_y << ", " << pca_span_t << std::endl;

    std::cout << "\tMax components: " << max_allowed_components << std::endl;
    std::cout << "\tComponent Threshold: " << significant_value_ratio << std::endl;
}
