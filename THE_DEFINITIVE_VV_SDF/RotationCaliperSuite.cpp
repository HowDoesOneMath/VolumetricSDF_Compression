#include "RotationCaliperSuite.h"

void RotationCaliperSuite::TestSingleRotation()
{
    
}

void RotationCaliperSuite::TestBlockPartition()
{
    GridDataStruct gds;

    gds.dim_x = grid_width_voxels;
    gds.dim_y = 2 * grid_width_voxels;
    gds.dim_z = grid_width_voxels;

    gds.unit_length = grid_width_meters / grid_width_voxels;

    gds.center_x = center.x();
    gds.center_y = center.y();
    gds.center_z = center.z();

    if (!sdf.InitializeGrid(gds))
    {
        std::cout << "COULDN'T INITIALIZE GRID!" << std::endl;
        return;
    }

    if (!mesh.ReadOBJ(test_mesh))
    {
        std::cout << "ERROR: mesh could not be loaded: " << test_mesh << std::endl;
        return;
    }

    mp.CreateUnionFindPartitions(mesh);
    mp.NegateInsignificantPartitions(mesh, mesh_maximum_artifact_size);
    mesh.ClearNegativeTriangles();

    if (!sdf.CastMeshUnsignedDistance(&mesh, shell_size))
    {
        std::cout << "ERROR: mesh could not be cast into SDF: " << test_mesh << std::endl;
        return;
    }

    std::cout << "Separating..." << std::endl;

    auto new_mesh = sdf.ExtractMeshQuantized();
    auto tri_groups = sdf.ExtractTriangleGroupsByBlock(*new_mesh, block_size);

    std::cout << "Total block groups: " << tri_groups->size() << std::endl;

    size_t section = 0;

    Eigen::Vector3i partitions = Eigen::Vector3i(
        sdf.GetDimX() / block_size.x(), 
        sdf.GetDimY() / block_size.y(),
        sdf.GetDimZ() / block_size.z()
        );

    size_t normal_groups = 0;

    size_t section_count = 0;
    for (size_t i = 0; i < tri_groups->size(); ++i)
    {
        section_count += ((*tri_groups)[i].size() > 0);
    }

    double uv_pad = approximate_pixel_uv_overlap / attribute_map_size;

    Eigen::Vector2i square_section_size;
    square_section_size.x() = std::ceil(sqrt(section_count));
    square_section_size.y() = square_section_size.x();
    Eigen::Vector2i current_section_coords = Eigen::Vector2i::Zero();
    Eigen::Vector2d section_spacing = Eigen::Vector2d(1.0 / square_section_size.x(), 1.0 / square_section_size.y());

    Eigen::Vector2d padded_spacing = section_spacing - Eigen::Vector2d::Ones() * 2.0 * uv_pad;

    new_mesh->uvs.indices.resize(new_mesh->vertices.indices.size());

    size_t dir_out = 0;
    size_t dir_in = 0;

    for (size_t x = 0; x < partitions.x(); ++x)
    {
        for (size_t y = 0; y < partitions.y(); ++y)
        {
            for (size_t z = 0; z < partitions.z(); ++z, ++section)
            {
                if ((*tri_groups)[section].size() <= 0)
                {
                    continue;
                }

                std::cout << current_section_coords.transpose() << " --- " << (section) << ", " << (*tri_groups)[section].size() << std::endl;

                auto mesh_section_by_block = new_mesh->ExtractSubmesh((*tri_groups)[section], VV_Mesh::VV_Mesh_Ignore_Flags::IGNORE_UVS);

                rc.ConstructRotationCaliperAtlas(*mesh_section_by_block, 
                    Eigen::Vector2d(section_spacing.x() * current_section_coords.x() + uv_pad, section_spacing.y() * current_section_coords.y() + uv_pad),
                    padded_spacing, minimum_normal_similarity, uv_pad);

                for (size_t i = 0; i < mesh_section_by_block->uvs.indices.size(); ++i)
                {
                    Eigen::Vector2d uv_p0 = mesh_section_by_block->uvs.elements[mesh_section_by_block->uvs.indices[i][0]];
                    Eigen::Vector2d uv_p1 = mesh_section_by_block->uvs.elements[mesh_section_by_block->uvs.indices[i][1]];
                    Eigen::Vector2d uv_p2 = mesh_section_by_block->uvs.elements[mesh_section_by_block->uvs.indices[i][2]];

                    Eigen::Vector2d uv_p10 = uv_p1 - uv_p0;
                    Eigen::Vector2d uv_p21 = uv_p2 - uv_p1;

                    double pseudo_uv_cross = uv_p10.x() * uv_p21.y() - uv_p10.y() * uv_p21.x();
                    bool is_out = (pseudo_uv_cross > 0);

                    dir_out += is_out;
                    dir_in += (1 - is_out);
                }

                std::cout << "Current UVS: " << mesh_section_by_block->uvs.indices.size() << ", ELEMS: " << mesh_section_by_block->uvs.elements.size() << std::endl;

                size_t last_uv_count = new_mesh->uvs.elements.size();
                Eigen::Vector3i last_uv_triplet = Eigen::Vector3i(last_uv_count, last_uv_count, last_uv_count);
                new_mesh->uvs.elements.resize(last_uv_count + mesh_section_by_block->uvs.elements.size());

                for (size_t i = 0; i < mesh_section_by_block->uvs.elements.size(); ++i)
                {
                    new_mesh->uvs.elements[i + last_uv_count] = mesh_section_by_block->uvs.elements[i];
                }

                for (size_t i = 0; i < mesh_section_by_block->uvs.indices.size(); ++i)
                {
                    new_mesh->uvs.indices[(*tri_groups)[section][i]] = mesh_section_by_block->uvs.indices[i] + last_uv_triplet;
                }

                ++current_section_coords.x();
                current_section_coords.y() += (current_section_coords.x() >= square_section_size.x());
                current_section_coords.x() *= (current_section_coords.x() < square_section_size.x());
            }
        }
    }

    std::cout << "Count OUT: " << dir_out << ", Count IN: " << dir_in << std::endl;

    std::cout << "Total normal groups: " << normal_groups << std::endl;

    texture.assign(test_texture.c_str());

    cimg_library::CImg<unsigned char> new_texture(attribute_map_size, attribute_map_size, 1, 3);
    new_texture.fill(0);

    if (!tr.Remap(mesh, *new_mesh, texture, new_texture, uv_pad, kernel_size, kernel_scale))
    {
        std::cout << "Could not remap!" << std::endl;
        return;
    }

    new_texture.save_png(output_texture.c_str());

    //separated_mesh.WriteOBJ(output_mesh);
    new_mesh->WriteOBJ(output_mesh);
}

void RotationCaliperSuite::run(int argc, char** argv)
{
	TestBlockPartition();
}
