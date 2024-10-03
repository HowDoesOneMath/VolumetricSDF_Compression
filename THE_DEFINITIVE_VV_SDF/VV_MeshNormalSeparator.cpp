#include "VV_MeshNormalSeparator.h"

std::shared_ptr<std::vector<std::vector<size_t>>> VV_MeshNormalSeparator::CreateVertexGroups(VV_Mesh& mesh)
{
    auto to_return = std::make_shared<std::vector<std::vector<size_t>>>();
    to_return->resize(mesh.vertices.elements.size());

    for (size_t i = 0; i < mesh.vertices.indices.size(); ++i)
    {
        (*to_return)[mesh.vertices.indices[i][0]].push_back(i);
        (*to_return)[mesh.vertices.indices[i][1]].push_back(i);
        (*to_return)[mesh.vertices.indices[i][2]].push_back(i);
    }

    return to_return;
}

std::shared_ptr<std::vector<Eigen::Vector3d>> VV_MeshNormalSeparator::CreateTriangleNormals(VV_Mesh& mesh, std::vector<bool>& precull_visitors)
{
    auto to_return = std::make_shared<std::vector<Eigen::Vector3d>>();

    to_return->resize(mesh.vertices.indices.size(), Eigen::Vector3d::Zero());

    Eigen::Vector3d* p0;
    Eigen::Vector3d* p1;
    Eigen::Vector3d* p2;

    Eigen::Vector3d p01;
    Eigen::Vector3d p02;

    Eigen::Vector3d normal;
    double sqr_norm;
    
    for (size_t i = 0; i < to_return->size(); ++i)
    {
        p0 = &mesh.vertices.elements[mesh.vertices.indices[i][0]];
        p1 = &mesh.vertices.elements[mesh.vertices.indices[i][1]];
        p2 = &mesh.vertices.elements[mesh.vertices.indices[i][2]];

        p01 = (*p0) - (*p1);
        p02 = (*p0) - (*p2);

        normal.x() = p01.y() * p02.z() - p01.z() * p02.y();
        normal.y() = p01.z() * p02.x() - p01.x() * p02.z();
        normal.z() = p01.x() * p02.y() - p01.y() * p02.x();

        sqr_norm = normal.squaredNorm();

        if (sqr_norm <= 0)
        {
            precull_visitors[i] = true;
            continue;
        }
        
        (*to_return)[i] = normal / sqrt(sqr_norm);
    }

    return to_return;
}

std::shared_ptr<std::vector<VV_NormalGroup>> VV_MeshNormalSeparator::SeparateMesh(VV_Mesh& to_separate, double separation_threshold)
{
    auto to_return = std::make_shared<std::vector<VV_NormalGroup>>();

    std::vector<bool> visited;
    visited.resize(to_separate.vertices.indices.size(), false);

    std::vector<size_t> presized_visitation_array;
    presized_visitation_array.resize(visited.size());
    size_t array_head = 0;
    size_t current_triangle;
    size_t current_vert;

    double normal_dot;

    auto normals = CreateTriangleNormals(to_separate, visited);

    auto links = CreateVertexGroups(to_separate);

    for (size_t i = 0; i < visited.size(); ++i)
    {
        if (visited[i]) {
            continue;
        }

        to_return->push_back(VV_NormalGroup());
        to_return->back().normal_direction = (*normals)[i];

        visited[i] = true;

        presized_visitation_array[0] = i;
        array_head = 1;

        while (array_head > 0)
        {
            --array_head;
            current_triangle = presized_visitation_array[array_head];

            to_return->back().triangles.push_back(current_triangle);

            for (int t = 0; t < 3; ++t)
            {
                current_vert = to_separate.vertices.indices[current_triangle][t];

                for (size_t j = 0; j < (*links)[current_vert].size(); ++j)
                {
                    if (visited[(*links)[current_vert][j]]) {
                        continue;
                    }

                    normal_dot = to_return->back().normal_direction.dot((*normals)[(*links)[current_vert][j]]);

                    if (normal_dot < separation_threshold) {
                        continue;
                    }

                    visited[(*links)[current_vert][j]] = true;

                    presized_visitation_array[array_head] = (*links)[current_vert][j];
                    ++array_head;
                }
            }
        }
    }

    //for (size_t i = 0; i < visited.si)

    return to_return;
}
