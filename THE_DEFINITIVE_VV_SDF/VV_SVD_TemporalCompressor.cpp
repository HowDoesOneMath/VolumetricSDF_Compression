#include "VV_SVD_TemporalCompressor.h"

size_t VV_SVD_TemporalCompressor::GetSignificantValueCount(double significant_value_ratio, size_t max_allowed_components)
{
    size_t to_return = 0;

    Eigen::VectorXd sigma_values = pca_enc.GetValues();

    size_t max_amount = std::min(max_allowed_components, (size_t)sigma_values.size());

    double max_allowed = significant_value_ratio * sigma_values.sum();

    for (; to_return < sigma_values.size(); ++to_return)
    {
        if (max_allowed <= 0 || to_return >= max_amount)
        {
            break;
        }

        max_allowed -= sigma_values[to_return];
    }

    return to_return;
}

void VV_SVD_TemporalCompressor::CopySignsOver()
{
    std::vector<size_t> file_locations_signs;
    file_locations_signs.resize(sad.total_frames);
    sfb.ReadArrayFromBuffer(file_locations_signs.data(), file_locations_signs.size());

    size_t signs_loc = sfb.GetWriterLocation();
    sfb.WriteArrayToBuffer(file_locations_signs.data(), file_locations_signs.size());

    std::vector<unsigned char> sign_array;

    for (size_t i = 0; i < file_locations_signs.size(); ++i)
    {
        sfb.SetReaderLocation(file_locations_signs[i]);
        sfb.ReadVectorFromBuffer(sign_array);
        file_locations_signs[i] = sfb.GetWriterLocation();
        sfb.WriteVectorToBuffer(sign_array);
    }

    size_t signs_end = sfb.GetWriterLocation();
    sfb.SetWriterLocation(signs_loc);
    sfb.WriteArrayToBuffer(file_locations_signs.data(), file_locations_signs.size());
    sfb.SetWriterLocation(signs_end);
}

void VV_SVD_TemporalCompressor::UseSigns(std::vector<unsigned char> sign_data)
{
    sdf.ApplySigns(sign_data);
}

void VV_SVD_TemporalCompressor::ConstructBlock(std::vector<size_t>& block_locations, std::vector<size_t>& svd_locations, 
    size_t svd_loc, size_t start_ind, size_t end_ind, double significant_value_ratio, size_t max_allowed_components)
{
    size_t pca_entry = 0;
    size_t significant_values;

#if TSVD_TIME_LOGGING
    tl.GetLogger(block_loading_time_logger_name)->StartTimer();
#endif

    for (size_t ind = start_ind; ind < end_ind; ind += sad.total_partitions, ++pca_entry)
    {
        sfb.SetReaderLocation(block_locations[ind]);

        lz_enc.ReadFromBuffer(sfb, block_buffer.data(), block_buffer.size());

        pca_enc.AddData(pca_entry, block_buffer.data());
    }

#if TSVD_TIME_LOGGING
    tl.GetLogger(block_loading_time_logger_name)->MarkTime();
    tl.GetLogger(SVD_time_logger_name)->StartTimer();
#endif

    double mat_avg = pca_enc.GetCenterOfData();
    pca_enc.AddValueToData(-mat_avg);

    pca_enc.ConductSVD();

#if TSVD_TIME_LOGGING
    tl.GetLogger(SVD_time_logger_name)->MarkTime();
    tl.GetLogger(matrix_writing_time_logger_name)->StartTimer();
#endif

    significant_values = GetSignificantValueCount(significant_value_ratio, max_allowed_components);

    Eigen::VectorXd sig_vals = pca_enc.GetValues();

    Eigen::MatrixXd u_mat = pca_enc.GetMatrixU();
    size_t u_mat_rows = u_mat.rows();
    Eigen::MatrixXd v_mat = pca_enc.GetMatrixV().transpose();
    size_t v_mat_cols = v_mat.cols();

    svd_locations[svd_loc] = sfb.GetWriterLocation();

    sfb.WriteObjectToBuffer(significant_values);
    sfb.WriteObjectToBuffer(mat_avg);

    if (significant_values <= 0)
    {
#if TSVD_TIME_LOGGING
        tl.GetLogger(matrix_writing_time_logger_name)->MarkTime();
#endif
        return;
    }

    std::vector<double> to_lz_compress;

    sfb.WriteObjectToBuffer(u_mat_rows);
    sfb.WriteObjectToBuffer(v_mat_cols);

    sfb.WriteVectorToBuffer(sig_vals, 0, significant_values);

    Eigen::MatrixXd sub_u_mat = u_mat.block(0, 0, u_mat_rows, significant_values);
    double u_mat_max_val = sub_u_mat.maxCoeff();
    double u_mat_min_val = sub_u_mat.minCoeff();
    Eigen::MatrixX<unsigned char> trunc_u_mat = TruncateMatrix<unsigned char>(sub_u_mat, u_mat_max_val, u_mat_min_val, matrix_truncation_maximum, 0);

    sfb.WriteObjectToBuffer(u_mat_max_val);
    sfb.WriteObjectToBuffer(u_mat_min_val);
    for (size_t r = 0; r < u_mat_rows; ++r)
    {
        sfb.WriteMatrixToBuffer(trunc_u_mat, r, 0, r + 1, significant_values);
    }

    Eigen::MatrixXd sub_v_mat = v_mat.block(0, 0, significant_values, v_mat_cols);
    double v_mat_max_val = sub_v_mat.maxCoeff();
    double v_mat_min_val = sub_v_mat.minCoeff();
    Eigen::MatrixX<unsigned char> trunc_v_mat = TruncateMatrix<unsigned char>(sub_v_mat, v_mat_max_val, v_mat_min_val, matrix_truncation_maximum, 0);

    sfb.WriteObjectToBuffer(v_mat_max_val);
    sfb.WriteObjectToBuffer(v_mat_min_val);
    sfb.WriteMatrixToBuffer(trunc_v_mat, 0, 0, significant_values, v_mat_cols);

#if TSVD_TIME_LOGGING
    tl.GetLogger(matrix_writing_time_logger_name)->MarkTime();
#endif
}

std::shared_ptr<VV_Mesh> VV_SVD_TemporalCompressor::ExtractSingleMesh(std::vector<size_t>& block_locations, std::vector<size_t>& sign_locations, size_t mesh_index_in_SVD)
{
    sdf.RefreshGrid();

    size_t partition_number = mesh_index_in_SVD / sad.frames_per_input_matrix;
    size_t place_in_partition = mesh_index_in_SVD % sad.frames_per_input_matrix;

    size_t exact_array_start = partition_number * sad.total_partitions;

    size_t array_loc;

    Eigen::MatrixXd extracted_u;
    Eigen::VectorXd extracted_s;
    Eigen::MatrixXd extracted_v;

    for (size_t x = 0, array_x = 0; x < sdf.GetDimX(); x += sad.block_size.x(), ++array_x)
    {
        for (size_t y = 0, array_y = 0; y < sdf.GetDimY(); y += sad.block_size.y(), ++array_y)
        {
            for (size_t z = 0, array_z = 0; z < sdf.GetDimZ(); z += sad.block_size.z(), ++array_z)
            {
                array_loc = partition_number * sad.partition_span_t + array_x * sad.partition_span_x + array_y * sad.partition_span_y + array_z;

                size_t sig_value_count;
                double avg;

                sfb.SetReaderLocation(block_locations[array_loc] & important_geometry_mask);

                sfb.ReadObjectFromBuffer(sig_value_count);
                sfb.ReadObjectFromBuffer(avg);

                if (sig_value_count <= 0)
                {
                    sdf.SetQuantizedBlock(x, y, z, sad.block_size.x(), sad.block_size.y(), sad.block_size.z(), avg + 0.1);
                    continue;
                }

                size_t u_mat_rows;
                size_t v_mat_cols;

                sfb.ReadObjectFromBuffer(u_mat_rows);
                sfb.ReadObjectFromBuffer(v_mat_cols);

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

                sfb.ReadVectorFromBuffer(d_values, 0, sig_value_count);

                double u_mat_max_val;
                double u_mat_min_val;

                double v_mat_max_val;
                double v_mat_min_val;

                sfb.ReadObjectFromBuffer(u_mat_max_val);
                sfb.ReadObjectFromBuffer(u_mat_min_val);

                size_t u_mat_location = sfb.GetReaderLocation();
                trunc_u_mat.setZero(1, sig_value_count);
                sfb.SetReaderLocation(u_mat_location + place_in_partition * sig_value_count * sizeof(unsigned char));
                sfb.ReadMatrixFromBuffer(trunc_u_mat, 0, 0, 1, sig_value_count);
                u_mat = UntruncateMatrix<unsigned char>(trunc_u_mat, u_mat_max_val, u_mat_min_val, matrix_truncation_maximum, 0);

                size_t v_mat_location = u_mat_location + u_mat_rows * sig_value_count * sizeof(unsigned char);
                sfb.SetReaderLocation(v_mat_location);

                sfb.ReadObjectFromBuffer(v_mat_max_val);
                sfb.ReadObjectFromBuffer(v_mat_min_val);

                trunc_v_mat.setZero(sig_value_count, v_mat_cols);
                sfb.ReadMatrixFromBuffer(trunc_v_mat, 0, 0, sig_value_count, v_mat_cols);
                v_mat = UntruncateMatrix<unsigned char>(trunc_v_mat, v_mat_max_val, v_mat_min_val, matrix_truncation_maximum, 0);

                d_mat = d_values.asDiagonal();

                Eigen::MatrixXd sdf_block = (u_mat * d_mat) * v_mat;

                sdf.InsertBlock(x, y, z, sad.block_size.x(), sad.block_size.y(), sad.block_size.z(), sdf_block, avg);
            }
        }
    }

#if ENCODE_SIGN_DATA
    size_t new_size_signs_lesser = sdf.GetQuantizedGridLength() / 8;
    size_t new_size_signs = new_size_signs_lesser + ((sdf.GetQuantizedGridLength() % 8) != 0);
    std::vector<unsigned char> compressed_signs(new_size_signs);

    sfb.SetReaderLocation(sign_locations[mesh_index_in_SVD]);
    lz_enc.ReadFromBuffer(sfb, compressed_signs.data(), compressed_signs.size());

    UseSigns(compressed_signs);
#endif
    auto to_return = sdf.ExtractMeshQuantized();

    return to_return;
}

void VV_SVD_TemporalCompressor::EncodeImportantBlocks(size_t start_t, size_t end_t, std::vector<size_t> &block_locations, std::vector<size_t> &sign_locations)
{
    size_t partition_number = start_t / sad.frames_per_input_matrix;
    size_t exact_array_start = partition_number * sad.total_partitions;
    size_t loc_index;

    for (size_t t = start_t; t < end_t; ++t)
    {
#if TSVD_TIME_LOGGING
        tl.GetLogger(important_blocks_logger_name)->StartTimer();
#endif
        auto new_mesh = ExtractSingleMesh(block_locations, sign_locations, t);

        auto tri_groups = sdf.ExtractTriangleGroupsByBlock(*new_mesh, sad.block_size);

        for (size_t i = 0; i < tri_groups->size(); ++i)
        {
            loc_index = i + exact_array_start;

            block_locations[loc_index] |= (((*tri_groups)[i].size() > 0) * important_geometry_flag);
        }

#if TSVD_TIME_LOGGING
        tl.GetLogger(important_blocks_logger_name)->MarkTime();
#endif
    }
}

std::shared_ptr<std::pair<std::vector<std::pair<size_t, Eigen::Vector2i>>, Eigen::Vector2i>> VV_SVD_TemporalCompressor::GetPatchInfo(
    std::vector<size_t>& block_locations)
{
    auto to_return = std::make_shared<std::pair<std::vector<std::pair<size_t, Eigen::Vector2i>>, Eigen::Vector2i>>();
    std::pair<size_t, Eigen::Vector2i> to_add;

    size_t total_important_blocks = 0;
    for (size_t i = 0; i < block_locations.size(); ++i)
    {
        total_important_blocks += ((block_locations[i] & important_geometry_flag) > 0);
    }

    double amnt_sqrt = ceil(sqrt((double)total_important_blocks));

    to_return->second.x() = amnt_sqrt;
    to_return->second.y() = to_return->second.x();

    size_t w = 0;
    size_t h = 0;

    for (size_t i = 0; i < block_locations.size(); ++i)
    {
        if ((block_locations[i] & important_geometry_flag) > 0)
        {
            to_add.first = i;
            to_add.second.x() = w;
            to_add.second.y() = h;
            to_return->first.push_back(to_add);

            ++w;

            h += (w >= to_return->second.x());
            w *= (w < to_return->second.x());
        }
    }

    return to_return;
}

void VV_SVD_TemporalCompressor::CreateUVs(VV_Mesh& target_mesh, std::vector<std::pair<size_t, Eigen::Vector2i>> &patch_locations, Eigen::Vector2d patch_spacing)
{
    auto tri_groups = sdf.ExtractTriangleGroupsByBlock(target_mesh, sad.block_size);

    size_t patch_index;
    Eigen::Vector2i patch_loc;

    Eigen::Vector2d uv_start;
    Eigen::Vector2d uv_dims;

    target_mesh.uvs.indices.resize(target_mesh.vertices.indices.size());

    for (size_t i = 0; i < patch_locations.size(); ++i)
    {
        patch_index = patch_locations[i].first;

        if ((*tri_groups)[patch_index].size() <= 0)
        {
            continue;
        }

        auto sub_mesh = target_mesh.ExtractSubmesh((*tri_groups)[patch_index], VV_Mesh::VV_Mesh_Ignore_Flags::IGNORE_UVS);

        patch_loc = patch_locations[i].second;

        uv_start.x() = (sad.patch_padding.x() + (double)patch_loc.x()) * patch_spacing.x();
        uv_start.y() = (sad.patch_padding.y() + (double)patch_loc.y()) * patch_spacing.y();

        uv_dims = sad.patch_scale.cwiseProduct(patch_spacing);

        rc.ConstructRotationCaliperAtlas(*sub_mesh, uv_start, uv_dims, sad.minimum_normal_similarity, sad.island_padding);

        size_t last_uv_count = target_mesh.uvs.elements.size();
        Eigen::Vector3i last_uv_triplet = Eigen::Vector3i(last_uv_count, last_uv_count, last_uv_count);
        target_mesh.uvs.elements.resize(last_uv_count + sub_mesh->uvs.elements.size());

        for (size_t i = 0; i < sub_mesh->uvs.elements.size(); ++i)
        {
            target_mesh.uvs.elements[i + last_uv_count] = sub_mesh->uvs.elements[i];
        }

        for (size_t i = 0; i < sub_mesh->uvs.indices.size(); ++i)
        {
            target_mesh.uvs.indices[(*tri_groups)[patch_index][i]] = sub_mesh->uvs.indices[i] + last_uv_triplet;
        }
    }
}

void VV_SVD_TemporalCompressor::DebugLocationsVector(std::vector<size_t>& to_debug, std::string message, size_t frequency, size_t offset)
{
    frequency = std::max(frequency, (size_t)1);

    for (size_t i = offset; i < to_debug.size(); i += frequency)
    {
        std::cout << message << " " << i << ": " << to_debug[i] << " (" << (to_debug[i] & important_geometry_mask) << ")" << std::endl;
    }
}

#if TSVD_TIME_LOGGING
void VV_SVD_TemporalCompressor::SetTimeLogFile(std::string time_log_name)
{
    tl.StartLogging(time_log_name);

    tl.CreateNewLogger(total_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(dummy_time_logger_name, new TimeLogger::IndependentLogger("\t"));

    tl.CreateNewLogger(header_intermediary_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(mesh_reading_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(mesh_cleaning_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(mesh_to_SDF_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(block_extraction_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(lz_encoding_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(sign_encoding_logger_name, new TimeLogger::IndependentLogger(""));

    tl.CreateNewLogger(header_TSVD_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(sign_copy_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(batch_TSVD_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(block_loading_time_logger_name, new TimeLogger::IndependentLogger("\t"));
    tl.CreateNewLogger(SVD_time_logger_name, new TimeLogger::IndependentLogger("\t"));
    tl.CreateNewLogger(matrix_writing_time_logger_name, new TimeLogger::IndependentLogger("\t"));

    tl.CreateNewLogger(header_textures_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(important_blocks_logger_name, new TimeLogger::IndependentLogger("\t"));
    tl.CreateNewLogger(patch_creation_texturing_logger_name, new TimeLogger::IndependentLogger("\t"));
    tl.CreateNewLogger(texture_batch_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(mesh_uv_creating_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(texture_remapping_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(texture_saving_logger_name, new TimeLogger::IndependentLogger(""));

    tl.CreateNewLogger(header_reconstruction_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(mesh_reconstruction_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(patch_creation_reconstruction_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(mesh_uv_reconstruction_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(mesh_obj_writing_logger_name, new TimeLogger::IndependentLogger(""));
}

void VV_SVD_TemporalCompressor::CloseTimeLogFile()
{
    tl.CloseLoggers();
}
#endif

bool VV_SVD_TemporalCompressor::Initialize(GridDataStruct& gds, Eigen::Vector3i block_size, size_t frames_per_batch, 
    double patch_padding, double island_padding, double minimum_normal_similarity)
{
    sad.gds = gds;
    sad.block_size = block_size;
    sad.frames_per_input_matrix = frames_per_batch;
    sad.patch_padding = Eigen::Vector2d(patch_padding, patch_padding);
    sad.island_padding = island_padding;
    sad.minimum_normal_similarity = minimum_normal_similarity;

    return true;
}

bool VV_SVD_TemporalCompressor::SaveIntermediaryFile(std::string root_folder, std::string intermediary_file_name, SequenceFinderDetails mesh_sf,
    double shell_buffer, double mesh_maximum_artifact_size)
{
#if TSVD_TIME_LOGGING
    tl.GetLogger(total_time_logger_name)->ResetTotalTime();
    tl.GetLogger(dummy_time_logger_name)->ResetTotalTime();

    tl.GetLogger(header_intermediary_time_logger_name)->ResetTotalTime();
    tl.GetLogger(mesh_reading_time_logger_name)->ResetTotalTime();
    tl.GetLogger(mesh_cleaning_time_logger_name)->ResetTotalTime();
    tl.GetLogger(mesh_to_SDF_time_logger_name)->ResetTotalTime();
    tl.GetLogger(block_extraction_logger_name)->ResetTotalTime();
    tl.GetLogger(lz_encoding_logger_name)->ResetTotalTime();
    tl.GetLogger(sign_encoding_logger_name)->ResetTotalTime();
#endif

    if (!sf.FindFiles(root_folder, mesh_sf))
    {
        std::cout << "Couldn't open input folder to read!" << std::endl;
        return false;
    }

    sad.total_frames = sf.files[mesh_sf.key].size();

    sad.RecalculateStats();

    sdf.ClearGrid();
    if (!sdf.InitializeGrid(sad.gds))
    {
        std::cout << "COULDN'T INITIALIZE GRID!" << std::endl;
        return false;
    }

    if (!sfb.OpenWriteBuffer(intermediary_file_name))
    {
        std::cout << "Couldn't open intermediary file to write!" << std::endl;
        return false;
    }

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_intermediary_time_logger_name)->StartTimer();
#endif

    sad.WriteToBuffer(sfb);

    std::vector<size_t> file_locations_blocks;
    file_locations_blocks.resize(sad.total_partitions * sad.total_frames);

#if ENCODE_SIGN_DATA
    std::vector<size_t> file_locations_signs;
    file_locations_signs.resize(sad.total_frames);
#endif

    size_t file_locations_blocks_start = sfb.GetWriterLocation();
    sfb.WriteArrayToBuffer(file_locations_blocks.data(), file_locations_blocks.size());

#if ENCODE_SIGN_DATA
    size_t file_locations_signs_start = sfb.GetWriterLocation();
    sfb.WriteArrayToBuffer(file_locations_signs.data(), file_locations_signs.size());
#endif

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_intermediary_time_logger_name)->MarkTime();
    std::cout << "Header time: " << (tl.GetLogger(header_intermediary_time_logger_name)->GetTime() * 0.000000001) << " seconds." << std::endl;
#endif

    size_t current_block_location = 0;

    block_buffer.resize(sad.total_block_size);

#if ENCODE_SIGN_DATA
    std::vector<unsigned char> signs;
#endif

    for (size_t t = 0; t < sad.total_frames; ++t)
    {
#if TSVD_TIME_LOGGING
        tl.GetLogger(total_time_logger_name)->StartTimer();
#endif

        std::cout << "Frame " << t << "..." << std::endl;

        if (!mesh.ReadOBJ(sf.files[mesh_sf.key][t]))
        {
            std::cout << "ERROR: mesh could not be loaded: " << sf.files[mesh_sf.key][t] << std::endl;
            return false;
        }

#if TSVD_TIME_LOGGING
        tl.GetLogger(mesh_reading_time_logger_name)->MarkTime();
        tl.GetLogger(mesh_cleaning_time_logger_name)->StartTimer();
#endif

        mesh.ClearUnreferencedElements();

        mp.CreateUnionFindPartitions(mesh);
        mp.NegateInsignificantPartitions(mesh, mesh_maximum_artifact_size);
        mesh.ClearNegativeTriangles();

#if TSVD_TIME_LOGGING
        tl.GetLogger(mesh_cleaning_time_logger_name)->MarkTime();
#endif

        std::cout << "\tBounding Box: " << mp.absolute_lower_bound.transpose() << " --- " << mp.absolute_upper_bound.transpose() << std::endl;

#if TSVD_TIME_LOGGING
        tl.GetLogger(mesh_to_SDF_time_logger_name)->StartTimer();
#endif

        //if (!sdf.CastMeshUnsignedDistance(&mesh, shell_buffer))
        if (!sdf.CastMeshUnsignedDistanceSampleAll(&mesh, shell_buffer))
        {
            std::cout << "ERROR: mesh could not be cast into SDF: " << sf.files[mesh_sf.key][t] << std::endl;
            return false;
        }

#if TSVD_TIME_LOGGING
        tl.GetLogger(mesh_to_SDF_time_logger_name)->MarkTime();
        std::cout << "Time converting mesh to SDF: " << (tl.GetLogger(mesh_to_SDF_time_logger_name)->GetTime() * 0.000000001) << " seconds." << std::endl;
        tl.GetLogger(block_extraction_logger_name)->SetTime(0);
        tl.GetLogger(lz_encoding_logger_name)->SetTime(0);
#endif

        for (size_t x = 0; x < sdf.GetDimX(); x += sad.block_size.x())
        {
            for (size_t y = 0; y < sdf.GetDimY(); y += sad.block_size.y())
            {
                for (size_t z = 0; z < sdf.GetDimZ(); z += sad.block_size.z())
                {
#if TSVD_TIME_LOGGING
                    tl.GetLogger(dummy_time_logger_name)->StartTimer();
#endif

                    sdf.ExtractBlock(x, y, z, sad.block_size.x(), sad.block_size.y(), sad.block_size.z(), block_buffer.data());

#if TSVD_TIME_LOGGING
                    tl.GetLogger(dummy_time_logger_name)->MarkTime();
                    tl.GetLogger(block_extraction_logger_name)->AddTime(tl.GetLogger(dummy_time_logger_name)->GetTime());
                    tl.GetLogger(dummy_time_logger_name)->StartTimer();
#endif

                    file_locations_blocks[current_block_location] = sfb.GetWriterLocation();
                    ++current_block_location;

                    lz_enc.SaveToBuffer(sfb, block_buffer.data(), block_buffer.size());
#if TSVD_TIME_LOGGING
                    tl.GetLogger(dummy_time_logger_name)->MarkTime();
                    tl.GetLogger(lz_encoding_logger_name)->AddTime(tl.GetLogger(dummy_time_logger_name)->GetTime());
#endif
                }
            }
        }

#if TSVD_TIME_LOGGING
        tl.GetLogger(block_extraction_logger_name)->MarkCustomTime(tl.GetLogger(block_extraction_logger_name)->GetTime());
        tl.GetLogger(lz_encoding_logger_name)->MarkCustomTime(tl.GetLogger(lz_encoding_logger_name)->GetTime());
#endif

#if ENCODE_SIGN_DATA
#if TSVD_TIME_LOGGING
        tl.GetLogger(sign_encoding_logger_name)->StartTimer();
#endif
        file_locations_signs[t] = sfb.GetWriterLocation();

        sdf.HarvestSigns(signs);
        lz_enc.SaveToBuffer(sfb, signs.data(), signs.size());

#if TSVD_TIME_LOGGING
        tl.GetLogger(sign_encoding_logger_name)->MarkTime();
#endif
#endif

#if TSVD_TIME_LOGGING
        tl.GetLogger(total_time_logger_name)->MarkTime();
        std::cout << "Time on this frame: " << (tl.GetLogger(total_time_logger_name)->GetTime() * 0.000000001) << " seconds." << std::endl;
#endif
    }

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_intermediary_time_logger_name)->StartTimer();
#endif

    size_t writer_eof = sfb.GetWriterLocation();

    sfb.SetWriterLocation(file_locations_blocks_start);
    sfb.WriteArrayToBuffer(file_locations_blocks.data(), file_locations_blocks.size());

#if ENCODE_SIGN_DATA
    sfb.SetWriterLocation(file_locations_signs_start);
    sfb.WriteArrayToBuffer(file_locations_signs.data(), file_locations_signs.size());
#endif

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_intermediary_time_logger_name)->MarkTime();
#endif

    sfb.SetWriterLocation(writer_eof);
    sfb.CloseWriteBuffer();

#if TSVD_TIME_LOGGING
    tl.PrintLoggerTotalTime(header_intermediary_time_logger_name, "Total header writing time: ");
    tl.PrintEmptyLine();
    
    tl.PrintTotalAndAverageAndGreatestTime(total_time_logger_name, "Total time", "Average time", "Greatest time", " of intermediary compression (minus header): ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(mesh_reading_time_logger_name, "Total time", "Average time", "Greatest time", " of mesh reading: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(mesh_cleaning_time_logger_name, "Total time", "Average time", "Greatest time", " of mesh cleaning: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(mesh_to_SDF_time_logger_name, "Total time", "Average time", "Greatest time", " of Mesh to SDF conversion: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(block_extraction_logger_name, "Total time", "Average time", "Greatest time", " of block extraction: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(lz_encoding_logger_name, "Total time", "Average time", "Greatest time", " of lz4 encoding/writing to file: ");
    tl.PrintEmptyLine();

#if ENCODE_SIGN_DATA
    tl.PrintTotalAndAverageAndGreatestTime(sign_encoding_logger_name, "Total time", "Average time", "Greatest time", " of Sign encoding/writing to file: ");
    tl.PrintEmptyLine();
#endif

    tl.PrintEmptyLine();
    tl.PrintSolidLine(40, '=');

#endif

    return true;
}

bool VV_SVD_TemporalCompressor::SaveFinalFile(std::string intermediary_file_name, std::string final_file_name, 
    double significant_value_ratio, size_t max_allowed_components)
{
#if TSVD_TIME_LOGGING
    tl.GetLogger(total_time_logger_name)->ResetTotalTime();
    tl.GetLogger(dummy_time_logger_name)->ResetTotalTime();

    tl.GetLogger(header_TSVD_time_logger_name)->ResetTotalTime();
    tl.GetLogger(sign_copy_time_logger_name)->ResetTotalTime();
    tl.GetLogger(batch_TSVD_time_logger_name)->ResetTotalTime();
#endif

    if (!sfb.OpenReadBuffer(intermediary_file_name))
    {
        std::cout << "Couldn't read intermediary file storage!" << std::endl;
        return false;
    }

    if (!sfb.OpenWriteBuffer(final_file_name))
    {
        std::cout << "Couldn't open PCA file to write!" << std::endl;
        return false;
    }

    sad.ReadFromBuffer(sfb);
    sad.WriteToBuffer(sfb);

    sdf.ClearGrid();
    sdf.InitializeGrid(sad.gds);

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_TSVD_time_logger_name)->StartTimer();
#endif

    block_buffer.resize(sad.total_block_size);

    std::vector<size_t> file_locations_blocks;
    file_locations_blocks.resize(sad.total_partitions * sad.total_frames);
    sfb.ReadArrayFromBuffer(file_locations_blocks.data(), file_locations_blocks.size());

    size_t total_t_partitions = sad.total_frames / sad.frames_per_input_matrix + ((sad.total_frames % sad.frames_per_input_matrix) > 0);
    std::vector<size_t> svd_block_locations(total_t_partitions * sad.total_partitions);
    size_t svd_block_locations_start = sfb.GetWriterLocation();
    sfb.WriteArrayToBuffer(svd_block_locations.data(), svd_block_locations.size());

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_TSVD_time_logger_name)->MarkTime();
#endif

#if ENCODE_SIGN_DATA
#if TSVD_TIME_LOGGING
    tl.GetLogger(sign_copy_time_logger_name)->StartTimer();
#endif
    CopySignsOver();
#if TSVD_TIME_LOGGING
    tl.GetLogger(sign_copy_time_logger_name)->MarkTime();
#endif
#endif

    pca_enc.Initialize(std::min(sad.frames_per_input_matrix, sad.total_frames), sad.total_block_size);

    size_t target_frame;

    size_t current_svd_block = 0;
    for (size_t t = 0; t < sad.total_frames; t += sad.frames_per_input_matrix)
    {
        target_frame = std::min(t + sad.frames_per_input_matrix, sad.total_frames);

        std::cout << "Batch " << t << " to " << target_frame << "..." << std::endl;

#if TSVD_TIME_LOGGING
        tl.GetLogger(batch_TSVD_time_logger_name)->StartTimer();

        tl.GetLogger(block_loading_time_logger_name)->ResetTotalTime();
        tl.GetLogger(SVD_time_logger_name)->ResetTotalTime();
        tl.GetLogger(matrix_writing_time_logger_name)->ResetTotalTime();
#endif

        for (size_t x = 0; x < sad.partitions.x(); ++x)
        {
            for (size_t y = 0; y < sad.partitions.y(); ++y)
            {
                for (size_t z = 0; z < sad.partitions.z(); ++z, ++current_svd_block)
                {
                    //std::cout << "\tPartition " << x << ", " << y << ", " << z << "..." << std::endl;

                    size_t start_ind = t * sad.partition_span_t + x * sad.partition_span_x + y * sad.partition_span_y + z;
                    size_t end_ind = target_frame * sad.partition_span_t + x * sad.partition_span_x + y * sad.partition_span_y + z;

                    ConstructBlock(file_locations_blocks, svd_block_locations, current_svd_block, start_ind, end_ind,
                        significant_value_ratio, max_allowed_components);
                }
            }
        }

        pca_enc.RefreshMatrix();

#if TSVD_TIME_LOGGING
        tl.GetLogger(batch_TSVD_time_logger_name)->MarkTime();
        tl.PrintLogger(batch_TSVD_time_logger_name, "SVD batch time (" + std::to_string(t) + " to " + std::to_string(target_frame) + "): ");
        tl.PrintEmptyLine();
        tl.PrintTotalAndAverageAndGreatestTime(block_loading_time_logger_name, "Total time", "Average time", "Greatest time", " of Block Loading: ");
        tl.PrintEmptyLine();
        tl.PrintTotalAndAverageAndGreatestTime(SVD_time_logger_name, "Total time", "Average time", "Greatest time", " of SVD: ");
        tl.PrintEmptyLine();
        tl.PrintTotalAndAverageAndGreatestTime(matrix_writing_time_logger_name, "Total time", "Average time", "Greatest time", " of UDV^T Writing: ");
        tl.PrintEmptyLine();
        tl.PrintEmptyLine();
#endif
    }

    size_t writer_eof = sfb.GetWriterLocation();

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_TSVD_time_logger_name)->StartTimer();
#endif

    sfb.SetWriterLocation(svd_block_locations_start);
    sfb.WriteArrayToBuffer(svd_block_locations.data(), svd_block_locations.size());
    sfb.SetWriterLocation(writer_eof);

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_TSVD_time_logger_name)->MarkTime();
#endif

    sfb.CloseWriteBuffer();
    sfb.CloseReadBuffer();

#if TSVD_TIME_LOGGING
    tl.PrintTotalAndAverageAndGreatestTime(batch_TSVD_time_logger_name, "Total time", "Average time", "Greatest time", " of Compression: ");
    tl.PrintEmptyLine();
    tl.PrintLoggerTotalTime(header_TSVD_time_logger_name, "Total time writing header: "); 
    tl.PrintEmptyLine();
#if ENCODE_SIGN_DATA
    tl.PrintLoggerTotalTime(sign_copy_time_logger_name, " Total time copying signs: ");
    tl.PrintEmptyLine();
#endif
    tl.PrintEmptyLine();
    tl.PrintSolidLine(40, '=');
#endif

    return true;
}

bool VV_SVD_TemporalCompressor::AugmentFinalFileWithTextureData(std::string root_folder, SequenceFinderDetails mesh_sf, SequenceFinderDetails texture_sf, 
    std::string final_file_name, std::string texture_tag, int digit_count, Eigen::Vector2i texture_dims, 
    double uv_epsilon, int kernel_size, double kernel_scale, int jpg_quality)
{
#if TSVD_TIME_LOGGING
    tl.GetLogger(total_time_logger_name)->ResetTotalTime();
    tl.GetLogger(dummy_time_logger_name)->ResetTotalTime();

    tl.GetLogger(header_textures_time_logger_name)->ResetTotalTime();
    tl.GetLogger(patch_creation_texturing_logger_name)->ResetTotalTime();
    tl.GetLogger(texture_batch_time_logger_name)->ResetTotalTime();
    tl.GetLogger(mesh_uv_creating_logger_name)->ResetTotalTime();
    tl.GetLogger(texture_remapping_logger_name)->ResetTotalTime();
    tl.GetLogger(texture_saving_logger_name)->ResetTotalTime();
#endif

    int pad_loops_hardcoded = 0;

    std::vector<SequenceFinderDetails> all_details;
    all_details.push_back(mesh_sf);
    all_details.push_back(texture_sf);

    if (!sf.FindFiles(root_folder, all_details))
    {
        std::cout << "Couldn't open input folder to read!" << std::endl;
        return false;
    }

    if (!sfb.OpenReadBuffer(final_file_name))
    {
        std::cout << "Couldn't read final file storage!" << std::endl;
        return false;
    }

    //size_t block_locations_offset = sfb.GetReaderLocation();

    sad.ReadFromBuffer(sfb);

    sdf.ClearGrid();
    if (!sdf.InitializeGrid(sad.gds))
    {
        std::cout << "COULDN'T INITIALIZE GRID!" << std::endl;
        return false;
    }


#if TSVD_TIME_LOGGING
    tl.GetLogger(header_textures_time_logger_name)->StartTimer();
#endif

    //block_locations_offset = (size_t)sfb.GetReaderLocation() - block_locations_offset;
    size_t block_locations_offset = sfb.GetReaderLocation();

    size_t total_t_partitions = sad.total_frames / sad.frames_per_input_matrix + ((sad.total_frames % sad.frames_per_input_matrix) > 0);
    std::vector<size_t> svd_block_locations(total_t_partitions * sad.total_partitions);
    sfb.ReadArrayFromBuffer(svd_block_locations.data(), svd_block_locations.size());

    std::vector<size_t> sign_locations(sad.total_frames);
#if ENCODE_SIGN_DATA
    sfb.ReadArrayFromBuffer(sign_locations.data(), sad.total_frames);
#endif

    std::vector<size_t> augmented_block_locations = svd_block_locations;

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_textures_time_logger_name)->MarkTime();
#endif

    std::string save_name;
    cimg_library::CImg<unsigned char> output_tex;

    Eigen::Vector2d patch_spacing;
    size_t total_important_blocks;
    size_t target_frame;


    for (size_t svd_group = 0; svd_group < sad.total_frames; svd_group += sad.frames_per_input_matrix)
    {
#if TSVD_TIME_LOGGING
        tl.GetLogger(total_time_logger_name)->StartTimer();
#endif
        target_frame = std::min(svd_group + sad.frames_per_input_matrix, sad.total_frames);

        std::cout << "Pre-processing texture data for batch " << svd_group << " to " << target_frame << "..." << std::endl;

#if TSVD_TIME_LOGGING
        tl.GetLogger(important_blocks_logger_name)->ResetTotalTime();
#endif

        EncodeImportantBlocks(svd_group, target_frame, augmented_block_locations, sign_locations);

#if TSVD_TIME_LOGGING
        tl.GetLogger(patch_creation_texturing_logger_name)->StartTimer();
#endif

        auto patch_info = GetPatchInfo(augmented_block_locations);

#if TSVD_TIME_LOGGING
        tl.GetLogger(patch_creation_texturing_logger_name)->MarkTime();
#endif

        patch_spacing = Eigen::Vector2d(1.0 / patch_info->second.x(), 1.0 / patch_info->second.y());

        std::cout << "Patch count: " << patch_info->first.size() << "/" << sad.total_partitions << std::endl;

        std::cout << "Texturing batch " << svd_group << " to " << target_frame << "..." << std::endl;

#if TSVD_TIME_LOGGING
        tl.GetLogger(texture_batch_time_logger_name)->StartTimer();
#endif

        for (size_t t = svd_group; t < target_frame; ++t)
        {
            std::cout << "\tFrame " << t << "..." << std::endl;

#if TSVD_TIME_LOGGING
            tl.GetLogger(mesh_uv_creating_logger_name)->StartTimer();
#endif

            auto new_mesh = ExtractSingleMesh(svd_block_locations, sign_locations, t);
            mesh.ReadOBJ(sf.files[mesh_sf.key][t]);

            CreateUVs(*new_mesh, patch_info->first, patch_spacing);

#if TSVD_TIME_LOGGING
            tl.GetLogger(mesh_uv_creating_logger_name)->MarkTime();
            tl.GetLogger(texture_remapping_logger_name)->StartTimer();
#endif

            output_tex.assign(texture_dims.x(), texture_dims.y(), 1, 3, 0);
            tex.assign(sf.files[texture_sf.key][t].c_str());

            if (!tr.Remap(mesh, *new_mesh, tex, output_tex, uv_epsilon, kernel_size, kernel_scale, pad_loops_hardcoded))
            {
                std::cout << "Could not remap " << t << "!" << std::endl;
            }

#if TSVD_TIME_LOGGING
            tl.GetLogger(texture_remapping_logger_name)->MarkTime();
            tl.GetLogger(texture_saving_logger_name)->StartTimer();
#endif

            save_name = texture_tag + "_" + GetNumberFixedLength(t, 6) + ".jpg";

            output_tex.save_jpeg(save_name.c_str(), jpg_quality);

#if TSVD_TIME_LOGGING
            tl.GetLogger(texture_saving_logger_name)->MarkTime();
#endif
        }

#if TSVD_TIME_LOGGING
        tl.GetLogger(texture_batch_time_logger_name)->MarkTime();
        tl.GetLogger(total_time_logger_name)->MarkTime();
        tl.PrintLogger(texture_batch_time_logger_name, "Texture batch time (" + std::to_string(svd_group) + " to " + std::to_string(target_frame) + "): ");
        tl.PrintEmptyLine();
        tl.PrintTotalAndAverageAndGreatestTime(important_blocks_logger_name, "Total time", "Average time", "Greatest time", " of Gathering Important Blocks: ");
        tl.PrintEmptyLine();
        tl.PrintLogger(patch_creation_texturing_logger_name, "Patch extraction time: ");
        tl.PrintEmptyLine();
#endif
    }

    sfb.CloseReadBuffer();

    if (!sfb.OpenWriteBufferNonTruncate(final_file_name))
    {
        std::cout << "Couldn't open final file to write!" << std::endl;
        return false;
    }

    std::cout << "Re-writing block locations..." << std::endl;

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_textures_time_logger_name)->StartTimer();
#endif

    sfb.SetWriterLocation(block_locations_offset);
    sfb.WriteArrayToBuffer(augmented_block_locations.data(), augmented_block_locations.size());

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_textures_time_logger_name)->MarkTime();
#endif

    sfb.SetWriterToEOF();
    sfb.CloseWriteBuffer();

#if TSVD_TIME_LOGGING
    tl.PrintLoggerTotalTime(header_textures_time_logger_name, "Total time to write header: ");
    tl.PrintEmptyLine();
    tl.PrintLoggerTotalTime(total_time_logger_name, "Total time to operate on textures: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(texture_batch_time_logger_name, "Total time", "Average time", "Greatest time", " of Texture Batches: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(mesh_uv_creating_logger_name, "Total time", "Average time per frame", "Greatest time per frame", " of Creating UVs: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(texture_remapping_logger_name, "Total time", "Average time per frame", "Greatest time per frame", " of Texture Remapping: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(texture_saving_logger_name, "Total time", "Average time per frame", "Greatest time per frame", " of Texture Saving: ");
    tl.PrintEmptyLine();
    tl.PrintEmptyLine();
    tl.PrintSolidLine(40, '=');
#endif

    return true;
}

bool VV_SVD_TemporalCompressor::ReconstructMeshes(std::string final_file_name, std::string output_mesh_tag, int digit_count)
{
#if TSVD_TIME_LOGGING
    tl.GetLogger(total_time_logger_name)->ResetTotalTime();
    tl.GetLogger(dummy_time_logger_name)->ResetTotalTime();

    tl.GetLogger(header_reconstruction_time_logger_name)->ResetTotalTime();
    tl.GetLogger(mesh_reconstruction_logger_name)->ResetTotalTime();
    tl.GetLogger(patch_creation_reconstruction_logger_name)->ResetTotalTime();
    tl.GetLogger(mesh_uv_reconstruction_logger_name)->ResetTotalTime();
    tl.GetLogger(mesh_obj_writing_logger_name)->ResetTotalTime();

#endif

    if (!sfb.OpenReadBuffer(final_file_name))
    {
        std::cout << "PROBLEMS OPENING PCA FILE" << std::endl;
        return false;
    }

    sad.ReadFromBuffer(sfb);

    sdf.ClearGrid();
    if (!sdf.InitializeGrid(sad.gds))
    {
        std::cout << "COULDN'T INITIALIZE GRID!" << std::endl;
        return false;
    }

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_reconstruction_time_logger_name)->StartTimer();
#endif

    size_t total_t_partitions = sad.total_frames / sad.frames_per_input_matrix + ((sad.total_frames % sad.frames_per_input_matrix) > 0);
    std::vector<size_t> svd_block_locations(total_t_partitions * sad.total_partitions);
    sfb.ReadArrayFromBuffer(svd_block_locations.data(), svd_block_locations.size());

    std::vector<size_t> sign_locations(sad.total_frames);
#if ENCODE_SIGN_DATA
    sfb.ReadArrayFromBuffer(sign_locations.data(), sad.total_frames);
#endif

#if TSVD_TIME_LOGGING
    tl.GetLogger(header_reconstruction_time_logger_name)->MarkTime();
#endif

    Eigen::Vector2d patch_spacing;

    for (size_t t = 0; t < sad.total_frames; ++t)
    {
        std::cout << "Frame " << t << "..." << std::endl;

#if TSVD_TIME_LOGGING
        tl.GetLogger(total_time_logger_name)->StartTimer();
        tl.GetLogger(mesh_reconstruction_logger_name)->StartTimer();
#endif

        auto new_mesh = ExtractSingleMesh(svd_block_locations, sign_locations, t);

#if TSVD_TIME_LOGGING
        tl.GetLogger(mesh_reconstruction_logger_name)->MarkTime();
        tl.GetLogger(patch_creation_reconstruction_logger_name)->StartTimer();
#endif

        auto patch_info = GetPatchInfo(svd_block_locations);

#if TSVD_TIME_LOGGING
        tl.GetLogger(patch_creation_reconstruction_logger_name)->MarkTime();
#endif

        std::cout << "Patch count: " << patch_info->first.size() << "/" << sad.total_partitions << std::endl;

#if TSVD_TIME_LOGGING
        tl.GetLogger(mesh_uv_reconstruction_logger_name)->StartTimer();
#endif

        patch_spacing = Eigen::Vector2d(1.0 / patch_info->second.x(), 1.0 / patch_info->second.y());
        CreateUVs(*new_mesh, patch_info->first, patch_spacing);

#if TSVD_TIME_LOGGING
        tl.GetLogger(mesh_uv_reconstruction_logger_name)->MarkTime();
        tl.GetLogger(mesh_obj_writing_logger_name)->StartTimer();
#endif

        std::string save_name = output_mesh_tag + "_" + GetNumberFixedLength(t, digit_count) + ".obj";

        new_mesh->WriteOBJ(save_name);

#if TSVD_TIME_LOGGING
        tl.GetLogger(mesh_obj_writing_logger_name)->MarkTime();
        tl.GetLogger(total_time_logger_name)->MarkTime();
#endif
    }

#if TSVD_TIME_LOGGING
    tl.PrintTotalAndAverageAndGreatestTime(total_time_logger_name, "Total time", "Average time", "Greatest time", " of Decompression: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(mesh_reconstruction_logger_name, "Total time", "Average time", "Greatest time", " of Mesh Reconstruction: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(patch_creation_reconstruction_logger_name, "Total time", "Average time per frame", "Greatest time per frame", " of Patch Reconstruction: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(mesh_uv_reconstruction_logger_name, "Total time", "Average time per frame", "Greatest time per frame", " of Mesh UVs: ");
    tl.PrintEmptyLine();
    tl.PrintTotalAndAverageAndGreatestTime(mesh_obj_writing_logger_name, "Total time", "Average time per frame", "Greatest time per frame", " of OBJ Writing: ");
#endif

    return true;
}

void VV_SVD_TemporalCompressor::SVD_AdditionalData::RecalculateStats()
{
    total_block_size = (size_t)block_size.x() * (size_t)block_size.y() * (size_t)block_size.z();

    partitions.x() = gds.dim_x / block_size.x();
    partitions.y() = gds.dim_y / block_size.y();
    partitions.z() = gds.dim_z / block_size.z();

    total_partitions = (size_t)partitions.x() * (size_t)partitions.y() * (size_t)partitions.z();

    partition_span_y = partitions.z();
    partition_span_x = partitions.y() * partition_span_y;
    partition_span_t = partitions.x() * partition_span_x;

    patch_scale = Eigen::Vector2d::Ones() - 2.0 * patch_padding;
}

void VV_SVD_TemporalCompressor::SVD_AdditionalData::WriteToBuffer(VV_SaveFileBuffer &buff)
{
    gds.WriteToBuffer(buff);

    buff.WriteObjectToBuffer(block_size.x());
    buff.WriteObjectToBuffer(block_size.y());
    buff.WriteObjectToBuffer(block_size.z());

    buff.WriteObjectToBuffer(frames_per_input_matrix);
    buff.WriteObjectToBuffer(total_frames);

    buff.WriteObjectToBuffer(patch_padding.x());
    buff.WriteObjectToBuffer(patch_padding.y());

    buff.WriteObjectToBuffer(island_padding);
}

void VV_SVD_TemporalCompressor::SVD_AdditionalData::ReadFromBuffer(VV_SaveFileBuffer &buff)
{
    gds.ReadFromBuffer(buff);

    buff.ReadObjectFromBuffer(block_size.x());
    buff.ReadObjectFromBuffer(block_size.y());
    buff.ReadObjectFromBuffer(block_size.z());

    buff.ReadObjectFromBuffer(frames_per_input_matrix);
    buff.ReadObjectFromBuffer(total_frames);

    buff.ReadObjectFromBuffer(patch_padding.x());
    buff.ReadObjectFromBuffer(patch_padding.y());

    buff.ReadObjectFromBuffer(island_padding);

    RecalculateStats();
}
