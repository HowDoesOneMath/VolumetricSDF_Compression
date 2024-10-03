#include "ArtifactCullingSuite.h"

void ArtifactCullingSuite::CreateComparisonMesh()
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

    if (!sdf.CastMeshUnsignedDistance(&input_mesh, shell_size))
    {
        std::cout << "ERROR: mesh could not be cast into SDF: " << input_mesh_name << std::endl;
        return;
    }

    auto output_mesh = sdf.ExtractMeshQuantized();
    sdf.RefreshGrid();

    mp.CreateUnionFindPartitions(input_mesh);
    mp.NegateInsignificantPartitions(input_mesh, mesh_maximum_artifact_size);
    input_mesh.ClearNegativeTriangles();

    if (!sdf.CastMeshUnsignedDistance(&input_mesh, shell_size))
    {
        std::cout << "ERROR: mesh could not be cast into SDF: " << input_mesh_name << std::endl;
        return;
    }

    output_mesh->ConcatenateMesh(*sdf.ExtractMeshQuantized(), offset_amount);

    output_mesh->WriteOBJ(output_mesh_name);
}

void ArtifactCullingSuite::run(int argc, char** argv)
{
    CreateComparisonMesh();
}
