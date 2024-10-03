#include "CGAL_RepairSuite.h"

//#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

void CGAL_RepairSuite::RepairTestMesh()
{
    if (!test_mesh.ReadOBJ(input_mesh_name))
    {
        std::cout << "Couldn't read input mesh: " << input_mesh_name << std::endl;
        return;
    }

    auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(test_mesh.vertices);
    auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, test_mesh.vertices);

    vcm.RepairMesh(*cgal_mesh);

    std::shared_ptr<VV_Mesh> new_mesh = std::make_shared<VV_Mesh>();
    vcm.CopyCGAL_To_VV_Attribute(*cgal_mesh, new_mesh->vertices);

    new_mesh->vertices.Concatenate(test_mesh.vertices, offset);

    new_mesh->WriteOBJ(output_mesh_name);
}

void CGAL_RepairSuite::run(int argc, char** argv)
{
    RepairTestMesh();
}
