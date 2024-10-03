#include "ShellSeparationSuite.h"

void ShellSeparationSuite::SeparateShell()
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

    if (!input_mesh.ReadOBJ(input_mesh_name))
    {
        std::cout << "ERROR: mesh could not be loaded: " << input_mesh_name << std::endl;
        return;
    }

    mp.CreateUnionFindPartitions(input_mesh);
    mp.NegateInsignificantPartitions(input_mesh, mesh_maximum_artifact_size);
    input_mesh.ClearNegativeTriangles();

    if (!sdf.CastMeshUnsignedDistance(&input_mesh, shell_size))
    {
        std::cout << "ERROR: mesh could not be cast into SDF: " << input_mesh_name << std::endl;
        return;
    }

    std::cout << "Separating..." << std::endl;

    auto new_mesh = sdf.ExtractMeshQuantized();
    auto tri_groups = sdf.ExtractTriangleGroupsByBlock(*new_mesh, block_size);

    Eigen::Vector3i partitions = Eigen::Vector3i(
        sdf.GetDimX() / block_size.x(),
        sdf.GetDimY() / block_size.y(),
        sdf.GetDimZ() / block_size.z()
    );

    size_t section = 0;

    size_t valid_section_count = 0;
    size_t normal_patch_count = 0;

    auto output_mesh = new_mesh->GetCopy();

    Eigen::Vector3d block_offset;
    Eigen::Vector3d normal_offset;

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

                ++valid_section_count;

                block_offset = Eigen::Vector3d(x, y, z) * block_spacing_diff;

                auto mesh_section_by_block = new_mesh->ExtractSubmesh((*tri_groups)[section], VV_Mesh::VV_Mesh_Ignore_Flags::IGNORE_UVS);

                auto normal_groups = rc.GetGroupsByNormal(*mesh_section_by_block, minimum_normal_similarity);

                for (size_t i = 0; i < normal_groups->size(); ++i)
                {
                    normal_offset = (*normal_groups)[i].first * normal_spacing_diff;

                    ++normal_patch_count;

                    output_mesh->ConcatenateMesh(*mesh_section_by_block->ExtractSubmesh((*normal_groups)[i].second), block_offset + normal_offset + concatenation_offset);
                }
            }
        }
    }

    std::cout << "Total block sections: " << valid_section_count << std::endl;

    std::cout << "Total normal patches: " << normal_patch_count << std::endl;

    output_mesh->WriteOBJ(output_mesh_name);
}

void ShellSeparationSuite::run(int argc, char** argv)
{
    SeparateShell();
}
