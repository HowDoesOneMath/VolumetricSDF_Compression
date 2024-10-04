#include "VSMC_Compressor.h"

void VSMC_Compressor::InitializeCompressor(double decimation_ratio, size_t output_attribute_width, size_t output_attribute_height, double gutter_amount, 
    int subdiv_count, size_t displacement_map_width, size_t displacement_block_size, int push_pull_kernel_size, double push_pull_kernel_scale,
    int draco_compression_speed, unsigned int jpg_quality)
{
    dec_ratio = decimation_ratio;
    width = output_attribute_width;
    height = output_attribute_height;
    gutter = gutter_amount;
    subdivision_count = subdiv_count;
    disp_width = displacement_map_width;
    b_size = displacement_block_size;
    ppk_size = push_pull_kernel_size;
    ppk_scale = push_pull_kernel_scale;

    enc_speed = draco_compression_speed;
    dec_speed = draco_compression_speed;
    dc = new DracoCompressor(quantization_options, enc_speed, dec_speed);

    jpg_q = jpg_quality;

    initialized = true;
}

#if VSMC_TIME_LOGGING
void VSMC_Compressor::SetTimeLogFile(std::string time_log_name)
{
    tl.StartLogging(time_log_name);

    tl.CreateNewLogger(total_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(frame_time_logger_name, new TimeLogger::IndependentLogger("\t"));

    tl.CreateNewLogger(cleaning_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(uvs_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(decimation_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(cull_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(normals_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(draco_compression_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(draco_decompression_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(remap_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(subdiv_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(displacement_time_logger_name, new TimeLogger::IndependentLogger(""));
    tl.CreateNewLogger(wavelet_time_logger_name, new TimeLogger::IndependentLogger(""));

}


void VSMC_Compressor::CloseTimeLogFile()
{
    tl.CloseLoggers();
}

#endif

#if USE_DOUBLE_DISPLACEMENTS
std::shared_ptr<std::pair<cimg_library::CImg<unsigned char>, cimg_library::CImg<double>>> VSMC_Compressor::CompressIntra(
    VV_Mesh& input_mesh, cimg_library::CImg<unsigned char>& input_texture, VV_SaveFileBuffer& sfb)
#else
std::shared_ptr<std::pair<cimg_library::CImg<unsigned char>, cimg_library::CImg<unsigned char>>> VSMC_Compressor::CompressIntra(
    VV_Mesh& input_mesh, cimg_library::CImg<unsigned char>& input_texture, VV_SaveFileBuffer& sfb)
#endif
{

#if VSMC_TIME_LOGGING
    tl.GetLogger(cleaning_time_logger_name)->StartTimer();
#endif

    mp.CreateUnionFindPartitions(input_mesh);
    mp.NegateInsignificantPartitions(input_mesh, mesh_maximum_artifact_size);
    input_mesh.ClearNegativeTriangles();
    input_mesh.ClearUnreferencedElements();

    //double irreg_thresh = 3.0;
    //input_mesh.DebugIrregularities(irreg_thresh);

#if USE_DOUBLE_DISPLACEMENTS
    auto to_return = std::make_shared<std::pair<cimg_library::CImg<unsigned char>, cimg_library::CImg<double>>>();
#else
    auto to_return = std::make_shared<std::pair<cimg_library::CImg<unsigned char>, cimg_library::CImg<unsigned char>>>();
#endif

    //std::cout << "Creating CGAL Marshall..." << std::endl;

    auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(input_mesh.vertices);
    //CGAL can automatically cull triangles that are improper when constructing its mesh, so this helps ensure that indices match between the original and CGAL
    auto i_map = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, input_mesh.vertices);


#if VSMC_TIME_LOGGING
    tl.GetLogger(cleaning_time_logger_name)->MarkTime();
    tl.GetLogger(decimation_time_logger_name)->StartTimer();
#endif

    auto cgal_decimated = vcm.DecimateCGAL_Mesh(*cgal_mesh, dec_ratio);

#if VSMC_TIME_LOGGING
    tl.GetLogger(decimation_time_logger_name)->MarkTime();
    tl.GetLogger(cull_time_logger_name)->StartTimer();
#endif

    vcm.CleanMeshCGAL(*cgal_mesh);
    auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

    last_intra = std::make_shared<VV_Mesh>();
    vcm.CopyCGAL_To_VV_Attribute(*cgal_decimated, last_intra->vertices);

    auto displacements = vcm.GetMeshDisplacements(*last_intra, input_mesh, *cgal_mesh, *i_map, *aabb_tree);
    last_intra->vertices.CullMassivelyDisplacedElements(*displacements, overly_displaced_threshold);

#if VSMC_TIME_LOGGING
    tl.GetLogger(cull_time_logger_name)->MarkTime();
    tl.GetLogger(uvs_time_logger_name)->StartTimer();
#endif

    if (!last_intra->GenerateNewUVsWithUVAtlas(width, height, gutter))
    {
        std::cout << "Could not make UVs!" << std::endl;
        return to_return;
    }

#if VSMC_TIME_LOGGING
    tl.GetLogger(uvs_time_logger_name)->MarkTime();
    tl.GetLogger(normals_time_logger_name)->StartTimer();
#endif

    if (input_mesh.normals.indices.size() > 0)
    {
        last_intra->normals.Clear();

        ////Normals interpolated from original mesh
        //last_intra->normals.indices = last_intra->vertices.indices;
        //last_intra->normals.elements.resize(last_intra->vertices.elements.size());
        //
        //Eigen::Vector3d barycentric_coords;
        //size_t hit_triangle_index;
        //
        //Eigen::Vector3i hit_triangle;
        //
        //for (size_t i = 0; i < last_intra->normals.elements.size(); ++i)
        //{
        //    vcm.FindClosestPointCGAL(*cgal_mesh, *aabb_tree, last_intra->vertices.elements[i], hit_triangle_index, barycentric_coords);
        //
        //    //hit_triangle = input_mesh.vertices.indices[(*i_map)[hit_triangle_index]];
        //    hit_triangle = input_mesh.normals.indices[(*i_map)[hit_triangle_index]];
        //
        //    last_intra->normals.elements[i] = input_mesh.normals.elements[hit_triangle.x()] * barycentric_coords.x() +
        //        input_mesh.normals.elements[hit_triangle.y()] * barycentric_coords.y() +
        //        input_mesh.normals.elements[hit_triangle.z()] * barycentric_coords.z();
        //
        //    last_intra->normals.elements[i] /= last_intra->normals.elements[i].norm();
        //}

        ////Per-triangle normals
        //last_intra->normals.elements.resize(last_intra->vertices.indices.size());
        //last_intra->normals.indices.resize(last_intra->vertices.indices.size());
        //
        //Eigen::Vector3d p10;
        //Eigen::Vector3d p20;
        //
        //Eigen::Vector3i* tri;
        //
        //for (size_t i = 0; i < last_intra->normals.elements.size(); ++i)
        //{
        //    tri = &(last_intra->vertices.indices[i]);
        //    p10 = last_intra->vertices.elements[tri->y()] - last_intra->vertices.elements[tri->x()];
        //    p20 = last_intra->vertices.elements[tri->z()] - last_intra->vertices.elements[tri->x()];
        //    last_intra->normals.elements[i] = CrossProduct(p10, p20).normalized();
        //    last_intra->normals.indices[i] = Eigen::Vector3i(i, i, i);
        //}

        //Normals derived by averaging cross products per vertex
        last_intra->normals.elements.resize(last_intra->vertices.elements.size(), Eigen::Vector3d::Zero());
        last_intra->normals.indices = last_intra->vertices.indices;

        Eigen::Vector3d p10;
        Eigen::Vector3d p20;

        Eigen::Vector3i* tri;

        Eigen::Vector3d cross_prod;

        for (size_t i = 0; i < last_intra->normals.indices.size(); ++i)
        {
            tri = &(last_intra->normals.indices[i]);
            p10 = last_intra->vertices.elements[tri->y()] - last_intra->vertices.elements[tri->x()];
            p20 = last_intra->vertices.elements[tri->z()] - last_intra->vertices.elements[tri->x()];

            cross_prod = CrossProduct(p10, p20);

            last_intra->normals.elements[tri->x()] += cross_prod;
            last_intra->normals.elements[tri->y()] += cross_prod;
            last_intra->normals.elements[tri->z()] += cross_prod;
        }

        for (size_t i = 0; i < last_intra->normals.elements.size(); ++i)
        {
            last_intra->normals.elements[i] /= last_intra->normals.elements[i].norm();
        }
    }

    std::vector<char> compression_buffer;


#if VSMC_TIME_LOGGING
    tl.GetLogger(normals_time_logger_name)->MarkTime();
    tl.GetLogger(draco_compression_time_logger_name)->StartTimer();
#endif

    dc->CompressMeshToVector(*last_intra, compression_buffer);

#if VSMC_TIME_LOGGING
    tl.GetLogger(draco_compression_time_logger_name)->MarkTime();
#endif

    sfb.WriteVectorToBuffer(compression_buffer);

#if VSMC_TIME_LOGGING
    tl.GetLogger(draco_decompression_time_logger_name)->StartTimer();
#endif
    
    dc->DecompressMeshFromVector(compression_buffer, *last_intra);

#if VSMC_TIME_LOGGING
    tl.GetLogger(draco_decompression_time_logger_name)->MarkTime();
    tl.GetLogger(remap_time_logger_name)->StartTimer();
#endif

    to_return->first.assign(width, height, 1, 3, 0);
    tr.Remap(input_mesh, *last_intra, input_texture, to_return->first, uv_epsilon, ppk_size, ppk_scale);
    //tr.FastRemap(input_mesh, *last_intra, input_texture, to_return->first, uv_epsilon, ppk_size, ppk_scale);

#if VSMC_TIME_LOGGING
    tl.GetLogger(remap_time_logger_name)->MarkTime();
    tl.GetLogger(subdiv_time_logger_name)->StartTimer();
#endif

    auto adjacencies = last_intra->SubdivideMeshAndGetAdjacencies(subdivision_count);

#if VSMC_TIME_LOGGING
    tl.GetLogger(subdiv_time_logger_name)->MarkTime();
    tl.GetLogger(displacement_time_logger_name)->StartTimer();
#endif

    displacements = vcm.GetMeshDisplacements(*last_intra, input_mesh, *cgal_mesh, *i_map, *aabb_tree);

#if VSMC_TIME_LOGGING
    tl.GetLogger(displacement_time_logger_name)->MarkTime();
#endif

    Eigen::Vector3d max_data_val = Eigen::Vector3d::Ones() * -DBL_MAX;
    Eigen::Vector3d min_data_val = Eigen::Vector3d::Ones() * DBL_MAX;

    size_t header_loc = sfb.GetWriterLocation();
    sfb.WriteObjectToBuffer(max_data_val[0]);
    sfb.WriteObjectToBuffer(max_data_val[1]);
    sfb.WriteObjectToBuffer(max_data_val[2]);
    sfb.WriteObjectToBuffer(min_data_val[0]);
    sfb.WriteObjectToBuffer(min_data_val[1]);
    sfb.WriteObjectToBuffer(min_data_val[2]);

    size_t starting_point;
    size_t midpoint_array_size;

    size_t p0_ind;
    size_t p1_ind;
    std::unordered_set<size_t> adjacent_points;
    size_t adjacent_point_count;

    std::vector<Eigen::Vector3d> midpoint_buffer;

#if VSMC_TIME_LOGGING
    tl.GetLogger(wavelet_time_logger_name)->StartTimer();
#endif

    for (int i = adjacencies->size() - 1; i >= 0; --i)
    {
        GetWaveletCoefficients((*adjacencies)[i].first, (*adjacencies)[i].second, *displacements);
    }

    for (size_t i = 0; i < displacements->size(); ++i)
    {
        min_data_val = min_data_val.cwiseMin((*displacements)[i]);
        max_data_val = max_data_val.cwiseMax((*displacements)[i]);
    }

    to_return->second.assign(disp_width, disp_width, 1, 3, 0);
    dp.FillImageBlocksRaster(to_return->second, b_size, *displacements, 0, displacement_max, min_data_val, max_data_val);

#if VSMC_TIME_LOGGING
    tl.GetLogger(wavelet_time_logger_name)->MarkTime();
#endif

    size_t end_loc = sfb.GetWriterLocation();
    sfb.SetWriterLocation(header_loc);
    sfb.WriteObjectToBuffer(max_data_val[0]);
    sfb.WriteObjectToBuffer(max_data_val[1]);
    sfb.WriteObjectToBuffer(max_data_val[2]);
    sfb.WriteObjectToBuffer(min_data_val[0]);
    sfb.WriteObjectToBuffer(min_data_val[1]);
    sfb.WriteObjectToBuffer(min_data_val[2]);
    sfb.SetWriterLocation(end_loc);

//#if VSMC_TIME_LOGGING
//    tl.PrintLoggerTotalTime(compression_time_logger_name, "Total time for frame (MINUS FILE WRITING): ");
//#endif

    return to_return;
}

#if USE_DOUBLE_DISPLACEMENTS
std::shared_ptr<VV_Mesh> VSMC_Compressor::DecompressIntra(size_t frame, std::vector<size_t>& file_locs, int subdiv_count,
    cimg_library::CImg<double> displacements, size_t block_size, VV_SaveFileBuffer& sfb)
#else
std::shared_ptr<VV_Mesh> VSMC_Compressor::DecompressIntra(size_t frame, std::vector<size_t>& file_locs, int subdiv_count,
    cimg_library::CImg<unsigned char> displacements, size_t block_size, VV_SaveFileBuffer& sfb)
#endif
{
    auto to_return = std::make_shared<VV_Mesh>();

    std::vector<char> draco_data;
    sfb.SetReaderLocation(file_locs[frame]);
    sfb.ReadVectorFromBuffer(draco_data);

#if VSMC_TIME_LOGGING
    tl.GetLogger(draco_decompression_time_logger_name)->StartTimer();
#endif

    dc->DecompressMeshFromVector(draco_data, *to_return);

#if VSMC_TIME_LOGGING
    tl.GetLogger(draco_decompression_time_logger_name)->MarkTime();
    tl.GetLogger(subdiv_time_logger_name)->StartTimer();
#endif

    auto adjacencies = to_return->SubdivideMeshAndGetAdjacencies(subdiv_count);

#if VSMC_TIME_LOGGING
    tl.GetLogger(subdiv_time_logger_name)->MarkTime();
#endif

    Eigen::Vector3d max_data_val;
    Eigen::Vector3d min_data_val;

    sfb.ReadObjectFromBuffer(max_data_val[0]);
    sfb.ReadObjectFromBuffer(max_data_val[1]);
    sfb.ReadObjectFromBuffer(max_data_val[2]);
    sfb.ReadObjectFromBuffer(min_data_val[0]);
    sfb.ReadObjectFromBuffer(min_data_val[1]);
    sfb.ReadObjectFromBuffer(min_data_val[2]);

    std::vector<Eigen::Vector3d> disp_vector(to_return->vertices.elements.size());

#if VSMC_TIME_LOGGING
    tl.GetLogger(wavelet_time_logger_name)->StartTimer();
#endif


    //RetrieveImageBlocksRaster<unsigned char>(displacements, block_size, disp_vector, 0, displacement_max, min_data_val, max_data_val);
    dp.RetrieveImageBlocksRaster(displacements, block_size, disp_vector, 0, displacement_max, min_data_val, max_data_val);

    for (int i = 0; i < adjacencies->size(); ++i)
    {
        InverseWaveletCoefficients((*adjacencies)[i].first, (*adjacencies)[i].second, disp_vector);
    }

#if VSMC_TIME_LOGGING
    tl.GetLogger(wavelet_time_logger_name)->MarkTime();
    tl.GetLogger(displacement_time_logger_name)->StartTimer();
#endif

    for (size_t i = 0; i < disp_vector.size(); ++i)
    {
        to_return->vertices.elements[i] += disp_vector[i];
    }

#if VSMC_TIME_LOGGING
    tl.GetLogger(displacement_time_logger_name)->MarkTime();
#endif

    //to_return_undisplaced->ConcatenateMesh(*to_return_unsubdiv, Eigen::Vector3d(0, 0, 0.7));
    //to_return->ConcatenateMesh(*to_return_undisplaced, Eigen::Vector3d(0, 0, 0.7));

    return to_return;
}

bool VSMC_Compressor::CompressSequence(std::string root_folder, SequenceFinderDetails mesh_sf, SequenceFinderDetails texture_sf, std::string output_file_name, 
    std::string output_texture_tag, std::string displacement_texture_tag, size_t starting_frame, size_t ending_frame, size_t digits_per_number)
{
    if (!initialized)
    {
        std::cout << "INITIALIZE BEFORE USING" << std::endl;
        return false;
    }

    if (!sfb.OpenWriteBuffer(output_file_name))
    {
        std::cout << "ERROR: Couldn't open file " << output_file_name << "!" << std::endl;
        return false;
    }

    std::vector<SequenceFinderDetails> details = { mesh_sf, texture_sf };

    if (!sf.FindFiles(root_folder, details))
    {
        return false;
    }

    if (sf.files[mesh_sf.key].size() != sf.files[texture_sf.key].size())
    {
        std::cout << "Mismatch of mesh and texture file count!\nMesh: " << sf.files[mesh_sf.key].size() << "\nTexture: " << sf.files[texture_sf.key].size() << std::endl;
        return false;
    }

    size_t item_count = std::min(sf.files[mesh_sf.key].size(), ending_frame);

    if (starting_frame >= item_count)
    {
        std::cout << "ERROR: Starting frame (" << starting_frame << ") needs to be less than ending frame ("
            << item_count << ")! (end frame is NOT inclusive, ending frame is capped to the total meshes found)" << std::endl;
    }

    VV_Mesh to_compress_mesh;
    cimg_library::CImg<unsigned char> to_compress_texture;

    std::string tex_file_name;
    std::string disp_file_name;

    size_t relative_ind = 0;

#if VSMC_TIME_LOGGING
    tl.GetLogger(total_time_logger_name)->ResetTotalTime();

    tl.GetLogger(cleaning_time_logger_name)->ResetTotalTime();
    tl.GetLogger(uvs_time_logger_name)->ResetTotalTime();
    tl.GetLogger(decimation_time_logger_name)->ResetTotalTime();
    tl.GetLogger(cull_time_logger_name)->ResetTotalTime();
    tl.GetLogger(normals_time_logger_name)->ResetTotalTime();
    tl.GetLogger(draco_compression_time_logger_name)->ResetTotalTime();
    tl.GetLogger(draco_decompression_time_logger_name)->ResetTotalTime();
    tl.GetLogger(remap_time_logger_name)->ResetTotalTime();
    tl.GetLogger(subdiv_time_logger_name)->ResetTotalTime();
    tl.GetLogger(displacement_time_logger_name)->ResetTotalTime();
    tl.GetLogger(wavelet_time_logger_name)->ResetTotalTime();

#endif

    sfb.WriteObjectToBuffer(subdivision_count);
    sfb.WriteObjectToBuffer(b_size);

    std::vector<size_t> file_locs;
    size_t file_locs_loc = sfb.GetWriterLocation();
    file_locs.resize(item_count - starting_frame);
    
    //Reserving space, will be re-written after
    sfb.WriteVectorToBuffer(file_locs);

    for (size_t i = starting_frame; i < item_count; ++i, ++relative_ind)
    {
        std::cout << "\tDEBUG frame " << i << " (" << relative_ind << ") \"" << sf.files[mesh_sf.key][i] << "\"..." << std::endl;

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->ResetTotalTime();
        tl.GetLogger(total_time_logger_name)->StartTimer();
        tl.GetLogger(frame_time_logger_name)->StartTimer();
#endif

        if (!to_compress_mesh.ReadOBJ(sf.files[mesh_sf.key][i]))
        {
            std::cout << "Couldn't read OBJ: " << sf.files[mesh_sf.key][i] << std::endl;
            return false;
        }

        to_compress_texture.assign(sf.files[texture_sf.key][i].c_str());

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->MarkTime();
        tl.PrintLogger(frame_time_logger_name, "Time to read files (FRAME " + std::to_string(i) + "): ");
#endif

        file_locs[relative_ind] = sfb.GetWriterLocation();

        std::cout << "Doing Compression..." << std::endl;

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->StartTimer();
#endif

        auto imgs = CompressIntra(to_compress_mesh, to_compress_texture, sfb);

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->MarkTime();
        tl.PrintLogger(frame_time_logger_name, "Time to compress (including file writing): ");
#endif

        if (imgs->first.width() == 0)
        {
            //continue;
            return false;
        }

        std::cout << "Saving tex... " << std::endl;

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->StartTimer();
#endif

        tex_file_name = output_texture_tag + "_" + GetNumberFixedLength(i, digits_per_number) + ".jpg";
        disp_file_name = displacement_texture_tag + "_" + GetNumberFixedLength(i, digits_per_number) + ".jpg";
        imgs->first.save_jpeg(tex_file_name.c_str(), jpg_q);
        imgs->second.save_jpeg(disp_file_name.c_str(), jpg_q);

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->MarkTime();
        tl.GetLogger(total_time_logger_name)->MarkTime();
        tl.PrintLogger(frame_time_logger_name, "Time to save displacement/albedo textures: ");
        tl.PrintLoggerTotalTime(frame_time_logger_name, "Total time to save frame: ");
        tl.PrintEmptyLine();
#endif
    }

    size_t eof_loc = sfb.GetWriterLocation();
    sfb.SetWriterLocation(file_locs_loc);
    sfb.WriteVectorToBuffer(file_locs);
    sfb.SetWriterLocation(eof_loc);
    sfb.CloseWriteBuffer();

#if VSMC_TIME_LOGGING

    tl.PrintTotalAndAverageAndGreatestTime(total_time_logger_name, "Total time", "Average time", "Greatest time", " of compression: ");

    tl.PrintTotalAndAverageAndGreatestTime(cleaning_time_logger_name, "Total time", "Average time", "Greatest time", " Cleaning Mesh: ");
    tl.PrintTotalAndAverageAndGreatestTime(uvs_time_logger_name, "Total time", "Average time", "Greatest time", " Making UVs: ");
    tl.PrintTotalAndAverageAndGreatestTime(decimation_time_logger_name, "Total time", "Average time", "Greatest time", " Decimating Mesh: ");
    tl.PrintTotalAndAverageAndGreatestTime(cull_time_logger_name, "Total time", "Average time", "Greatest time", " Culling Mesh: ");
    tl.PrintTotalAndAverageAndGreatestTime(normals_time_logger_name, "Total time", "Average time", "Greatest time", " Making Normals: ");
    tl.PrintTotalAndAverageAndGreatestTime(draco_compression_time_logger_name, "Total time", "Average time", "Greatest time", " Compressing w/Draco: ");
    tl.PrintTotalAndAverageAndGreatestTime(draco_decompression_time_logger_name, "Total time", "Average time", "Greatest time", " Decompressing w/Draco: ");
    tl.PrintTotalAndAverageAndGreatestTime(remap_time_logger_name, "Total time", "Average time", "Greatest time", " Remapping Texture: ");
    tl.PrintTotalAndAverageAndGreatestTime(subdiv_time_logger_name, "Total time", "Average time", "Greatest time", " Subdividing Mesh: ");
    tl.PrintTotalAndAverageAndGreatestTime(displacement_time_logger_name, "Total time", "Average time", "Greatest time", " Getting Displacements: ");
    tl.PrintTotalAndAverageAndGreatestTime(wavelet_time_logger_name, "Total time", "Average time", "Greatest time", " Calculating Wavelets: ");

    tl.PrintEmptyLine();
    tl.PrintEmptyLine();
    tl.PrintEmptyLine();
#endif

    return true;
}

bool VSMC_Compressor::DecompressSequence(std::string input_file_name, std::string displacements_folder, SequenceFinderDetails displacement_texture_sf,
    std::string output_mesh_tag, size_t digits_per_number)
{
    if (!sfb.OpenReadBuffer(input_file_name))
    {
        std::cout << "ERROR: Couldn't open file " << input_file_name << "!" << std::endl;
        return false;
    }

    if (!sf.FindFiles(displacements_folder, displacement_texture_sf))
    {
        std::cout << "ERROR: Couldn't find displacements at " << displacements_folder << "!" << std::endl;
        return false;
    }

#if USE_DOUBLE_DISPLACEMENTS
    cimg_library::CImg<double> displacement_tex;
#else
    cimg_library::CImg<unsigned char> displacement_tex;
#endif

    int expected_subdiv_count;
    size_t expected_block_size;

#if VSMC_TIME_LOGGING
    tl.GetLogger(total_time_logger_name)->ResetTotalTime();

    tl.GetLogger(draco_decompression_time_logger_name)->ResetTotalTime();
    tl.GetLogger(subdiv_time_logger_name)->ResetTotalTime();
    tl.GetLogger(displacement_time_logger_name)->ResetTotalTime();
    tl.GetLogger(wavelet_time_logger_name)->ResetTotalTime();
#endif

    sfb.ReadObjectFromBuffer(expected_subdiv_count);
    sfb.ReadObjectFromBuffer(expected_block_size);

    std::vector<size_t> file_locs;
    sfb.ReadVectorFromBuffer(file_locs);

    for (size_t i = 0; i < file_locs.size(); ++i)
    {
        std::cout << "Decompressing frame " << i << "..." << std::endl;

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->ResetTotalTime();
        tl.GetLogger(total_time_logger_name)->StartTimer();
        tl.GetLogger(frame_time_logger_name)->StartTimer();
#endif

        displacement_tex.assign(sf.files[displacement_texture_sf.key][i].c_str());

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->MarkTime();
        tl.PrintLogger(frame_time_logger_name, "Time to read displacements: ");
        tl.GetLogger(frame_time_logger_name)->StartTimer();
#endif

        auto new_mesh = DecompressIntra(i, file_locs, expected_subdiv_count, displacement_tex, expected_block_size, sfb);

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->MarkTime();
        tl.PrintLogger(frame_time_logger_name, "Time to decompress: ");
        tl.GetLogger(frame_time_logger_name)->StartTimer();
#endif

        std::string save_name = output_mesh_tag + "_" + GetNumberFixedLength(i, digits_per_number) + ".obj";
        
        new_mesh->WriteOBJ(save_name);

#if VSMC_TIME_LOGGING
        tl.GetLogger(frame_time_logger_name)->MarkTime();
        tl.GetLogger(total_time_logger_name)->MarkTime();
        tl.PrintLogger(frame_time_logger_name, "Time to write obj: ");
        tl.PrintLoggerTotalTime(frame_time_logger_name, "Total time to decompress frame: ");
        tl.PrintEmptyLine();
#endif
    }

#if VSMC_TIME_LOGGING
    tl.PrintTotalAndAverageAndGreatestTime(total_time_logger_name, "Total time", "Average time", "Greatest time", " of decompression: ");

    tl.PrintTotalAndAverageAndGreatestTime(draco_decompression_time_logger_name, "Total time", "Average time", "Greatest time", " for Draco Decompression: ");
    tl.PrintTotalAndAverageAndGreatestTime(subdiv_time_logger_name, "Total time", "Average time", "Greatest time", " for Subdivision: ");
    tl.PrintTotalAndAverageAndGreatestTime(wavelet_time_logger_name, "Total time", "Average time", "Greatest time", " for Wavelets: ");
    tl.PrintTotalAndAverageAndGreatestTime(displacement_time_logger_name, "Total time", "Average time", "Greatest time", " for Displacement: ");

    tl.PrintEmptyLine();
    tl.PrintEmptyLine();
    tl.PrintEmptyLine();
#endif

    return true;
}

void VSMC_Compressor::GetWaveletCoefficients(size_t index_offset, std::vector<std::tuple<size_t, size_t, std::unordered_set<size_t>>>& adjacencies, 
    std::vector<Eigen::Vector3d>& displacements)
{
    size_t adjacency_count = adjacencies.size();

    //std::vector<Eigen::Vector3d> midpoint_buffer;
    //
    //midpoint_buffer.resize(adjacency_count);

    for (size_t i = 0; i < adjacency_count; ++i)
    {
        size_t p0_ind = std::get<0>(adjacencies[i]);
        size_t p1_ind = std::get<1>(adjacencies[i]);

        displacements[i + index_offset] -= ((displacements[p0_ind] + displacements[p1_ind]) * 0.5);
    }

    //for (size_t i = 0; i < adjacency_count; ++i)
    //{
    //    std::unordered_set<size_t> adjacent_points = std::get<2>(adjacencies[i]);
    //    size_t adjacent_point_count = adjacent_points.size();
    //
    //    midpoint_buffer[i] = Eigen::Vector3d::Zero();
    //    for (auto adj : adjacent_points)
    //    {
    //        midpoint_buffer[i] += displacements[adj];
    //    }
    //
    //    midpoint_buffer[i] /= adjacent_point_count;
    //}
    //
    //for (size_t i = 0; i < adjacency_count; ++i)
    //{
    //    displacements[i + index_offset] += midpoint_buffer[i];
    //}
}

void VSMC_Compressor::InverseWaveletCoefficients(size_t index_offset, std::vector<std::tuple<size_t, size_t, std::unordered_set<size_t>>>& adjacencies, std::vector<Eigen::Vector3d>& displacements)
{
    size_t adjacency_count = adjacencies.size();

    //std::vector<Eigen::Vector3d> midpoint_buffer;
    //
    //midpoint_buffer.resize(adjacency_count);
    //
    //for (size_t i = 0; i < adjacency_count; ++i)
    //{
    //    std::unordered_set<size_t> adjacent_points = std::get<2>(adjacencies[i]);
    //    size_t adjacent_point_count = adjacent_points.size();
    //
    //    midpoint_buffer[i] = Eigen::Vector3d::Zero();
    //    for (auto adj : adjacent_points)
    //    {
    //        midpoint_buffer[i] += displacements[adj];
    //    }
    //
    //    midpoint_buffer[i] /= adjacent_point_count;
    //}

    for (size_t i = 0; i < adjacency_count; ++i)
    {
        size_t p0_ind = std::get<0>(adjacencies[i]);
        size_t p1_ind = std::get<1>(adjacencies[i]);

        displacements[i + index_offset] += ((displacements[p0_ind] + displacements[p1_ind]) * 0.5);

        //displacements[i + index_offset] -= midpoint_buffer[i];
    }
}
