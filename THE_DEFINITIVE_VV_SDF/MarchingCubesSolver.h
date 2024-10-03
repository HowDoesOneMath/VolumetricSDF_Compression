#pragma once

#include <Eigen/Core>

#include <iostream>

#include "VV_Mesh.h"
#include "GridDataStruct.h"
#include "MortonOrderer.h"

#include "MarchingTables.h"

class MarchingCubesSolver
{
    MortonOrderer mo;
public:
    struct MarchingVoxelData
    {
        MarchingVoxelData()
        {
            vert_indices_on_edges.x() = -1;
            vert_indices_on_edges.y() = -1;
            vert_indices_on_edges.z() = -1;
        }

        size_t grid_location = 0;
        size_t triangle_indices_start = 0;        
        
        Eigen::Vector3i vert_indices_on_edges;
        int triangle_count = 0;
    };

    struct MarchingVertexData
    {
        size_t parent_voxel = 0;

        char axis = 0;
        //bool facing_positive = false;
        //bool facing_negative = false;
    };

    std::vector<size_t> inds;
    //Maps MarchingVoxelData to grid indices, per important SDF point
    std::vector<MarchingVoxelData> mc_data;
    //Maps MarchingVertexData to MarchingVoxelData, per vertex
    std::vector<MarchingVertexData> mc_verts;

private:
    GridDataStruct gds;
    size_t data_size;

    Eigen::Vector3i null_index = Eigen::Vector3i(-1, -1, -1);

    Eigen::Vector3d grid_lower_bound;

    Eigen::Vector3i dim_minus_one;

    size_t span_x;
    size_t span_y;
    size_t span_z;

    void CreateNewIndex(size_t location)
    {
        inds[location] = mc_data.size();

        mc_data.push_back(MarchingVoxelData());

        mc_data.back().grid_location = location;
    }

    template<typename T>
    size_t ExtractEdge(T* data, VV_Mesh& to_return, size_t* corners_array, Eigen::Vector3d* positions_array, 
        size_t index_0, size_t index_1, char edge_direction, double threshold) {

        if (mc_data[inds[corners_array[index_0]]].vert_indices_on_edges[edge_direction] < 0)
        {
            double v0 = abs((double)data[corners_array[index_0]] - threshold);
            double v1 = abs((double)data[corners_array[index_1]] - threshold);

            double t = v0 / (v0 + v1);

            mc_data[inds[corners_array[index_0]]].vert_indices_on_edges[edge_direction] = to_return.vertices.elements.size();
            to_return.vertices.elements.push_back(positions_array[index_0] * (1.0 - t) + positions_array[index_1] * t);

            mc_verts.push_back(MarchingVertexData());
            mc_verts.back().axis = edge_direction;
            mc_verts.back().parent_voxel = inds[corners_array[index_0]];

            //mc_verts.back().facing_negative = (data[corners_array[index_0]] > data[corners_array[index_1]]);
            //mc_verts.back().facing_positive = !mc_verts.back().facing_negative;
        }

        return mc_data[inds[corners_array[index_0]]].vert_indices_on_edges[edge_direction];
    }

    template<typename T>
    size_t ExtractEdgeWithSqrtAdjustedValues(T* data, VV_Mesh& to_return, size_t* corners_array, Eigen::Vector3d* positions_array, 
        size_t index_0, size_t index_1, int edge_direction, double adjustment) {

        if (mc_data[inds[corners_array[index_0]]].vert_indices_on_edges[edge_direction] < 0)
        {
            double v0 = abs(sqrt((double)data[corners_array[index_0]]) - adjustment);
            double v1 = abs(sqrt((double)data[corners_array[index_1]]) - adjustment);

            double t = v0 / (v0 + v1);

            mc_data[inds[corners_array[index_0]]].vert_indices_on_edges[edge_direction] = to_return.vertices.elements.size();
            to_return.vertices.elements.push_back(positions_array[index_0] * (1.0 - t) + positions_array[index_1] * t);

            mc_verts.push_back(MarchingVertexData());
            mc_verts.back().axis = edge_direction;
            mc_verts.back().parent_voxel = inds[corners_array[index_0]];
                           
            //mc_verts.back().facing_negative = (data[corners_array[index_0]] > data[corners_array[index_1]]);
            //mc_verts.back().facing_positive = !mc_verts.back().facing_negative;
        }


        return mc_data[inds[corners_array[index_0]]].vert_indices_on_edges[edge_direction];
    }

public:
    void ResetIndices() {
        for (size_t i = 0; i < mc_data.size(); ++i)
        {
            inds[mc_data[i].grid_location] = SIZE_MAX;
        }

        mc_data.clear();
        mc_verts.clear();
    }

    MarchingCubesSolver(GridDataStruct &gds) {
        this->gds = gds;

        data_size = gds.dim_x * gds.dim_y * gds.dim_z;
        dim_minus_one = Eigen::Vector3i(gds.dim_x - 1, gds.dim_y - 1, gds.dim_z - 1);

        inds.resize(data_size, SIZE_MAX);
        //edge_indices.resize(data_size, null_index);

        span_z = 1;
        span_y = gds.dim_z;
        span_x = gds.dim_y * span_y;

        grid_lower_bound = Eigen::Vector3d(gds.center_x, gds.center_y, gds.center_z) - Eigen::Vector3d(dim_minus_one[0], dim_minus_one[1], dim_minus_one[2]) * 0.5 * gds.unit_length;

        std::cout << data_size << "\n" << 
            dim_minus_one.transpose() << "\n" <<
            span_x << "\n" <<
            span_y << "\n" <<
            std::endl;
    }

    template<typename T>
	std::shared_ptr<VV_Mesh> ExtractMeshFromGrid(T* data, double threshold);

    template<typename T>
    std::shared_ptr<VV_Mesh> ExtractIsosurfaceMeshFromGridWithSquaredValues(T* data, double sqrt_of_target_value);
};

template<typename T>
inline std::shared_ptr<VV_Mesh> MarchingCubesSolver::ExtractMeshFromGrid(T* data, double threshold)
{
    ResetIndices();

    auto to_return = std::make_shared<VV_Mesh>();

    size_t corners[8];

    Eigen::Vector3d positions[8];
    Eigen::Vector3d right = Eigen::Vector3d(gds.unit_length, 0, 0);
    Eigen::Vector3d up = Eigen::Vector3d(0, gds.unit_length, 0);
    Eigen::Vector3d forward = Eigen::Vector3d(0, 0, gds.unit_length);

    size_t edges[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };

    size_t visited_cells = 0;

    for (size_t x = 0; x < dim_minus_one[0]; ++x)
    {
        for (size_t y = 0; y < dim_minus_one[1]; ++y)
        {
            for (size_t z = 0; z < dim_minus_one[2]; ++z)
            {
                corners[0] = z + y * span_y + x * span_x;
                corners[4] = corners[0] + span_z;
                corners[2] = corners[0] + span_y;
                corners[6] = corners[2] + span_z;
                corners[1] = corners[0] + span_x;
                corners[5] = corners[1] + span_z;
                corners[3] = corners[1] + span_y;
                corners[7] = corners[3] + span_z;

                int index = 0;

                index |= 1 * (data[corners[0]] < threshold);
                index |= 2 * (data[corners[1]] < threshold);
                index |= 8 * (data[corners[2]] < threshold);
                index |= 4 * (data[corners[3]] < threshold);
                index |= 16 * (data[corners[4]] < threshold);
                index |= 32 * (data[corners[5]] < threshold);
                index |= 128 * (data[corners[6]] < threshold);
                index |= 64 * (data[corners[7]] < threshold);

                index = 255 - index;

                if (!MarchingTables::edge_table[index])
                    continue;

                for (int i = 0; i < 8; ++i)
                {
                    if (inds[corners[i]] == SIZE_MAX)
                    {
                        CreateNewIndex(corners[i]);
                    }
                }


                ++visited_cells;

                positions[0] = Eigen::Vector3d(x, y, z) * gds.unit_length + grid_lower_bound;
                positions[4] = positions[0] + forward;
                positions[2] = positions[0] + up;
                positions[6] = positions[2] + forward;
                positions[1] = positions[0] + right;
                positions[5] = positions[1] + forward;
                positions[3] = positions[1] + up;
                positions[7] = positions[3] + forward;

                if ((MarchingTables::edge_table[index] & 1) > 0)
                    edges[0] = ExtractEdge(data, *to_return, corners, positions, 0, 1, 0, threshold);
                if ((MarchingTables::edge_table[index] & 2) > 0)
                    edges[1] = ExtractEdge(data, *to_return, corners, positions, 1, 3, 1, threshold);
                if ((MarchingTables::edge_table[index] & 4) > 0)
                    edges[2] = ExtractEdge(data, *to_return, corners, positions, 2, 3, 0, threshold);
                if ((MarchingTables::edge_table[index] & 8) > 0)
                    edges[3] = ExtractEdge(data, *to_return, corners, positions, 0, 2, 1, threshold);
                if ((MarchingTables::edge_table[index] & 16) > 0)
                    edges[4] = ExtractEdge(data, *to_return, corners, positions, 4, 5, 0, threshold);
                if ((MarchingTables::edge_table[index] & 32) > 0)
                    edges[5] = ExtractEdge(data, *to_return, corners, positions, 5, 7, 1, threshold);
                if ((MarchingTables::edge_table[index] & 64) > 0)
                    edges[6] = ExtractEdge(data, *to_return, corners, positions, 6, 7, 0, threshold);
                if ((MarchingTables::edge_table[index] & 128) > 0)
                    edges[7] = ExtractEdge(data, *to_return, corners, positions, 4, 6, 1, threshold);
                if ((MarchingTables::edge_table[index] & 256) > 0)
                    edges[8] = ExtractEdge(data, *to_return, corners, positions, 0, 4, 2, threshold);
                if ((MarchingTables::edge_table[index] & 512) > 0)
                    edges[9] = ExtractEdge(data, *to_return, corners, positions, 1, 5, 2, threshold);
                if ((MarchingTables::edge_table[index] & 1024) > 0)
                    edges[10] = ExtractEdge(data, *to_return, corners, positions, 3, 7, 2, threshold);
                if ((MarchingTables::edge_table[index] & 2048) > 0)
                    edges[11] = ExtractEdge(data, *to_return, corners, positions, 2, 6, 2, threshold);

                auto tri_table_seg = MarchingTables::tri_table[index];

                mc_data[inds[corners[0]]].triangle_indices_start = to_return->vertices.indices.size();

                //Adding triangles to the mesh
                for (int i = 0; tri_table_seg[i] != -1; i += 3)
                {
                    to_return->vertices.indices.push_back(Eigen::Vector3i(edges[tri_table_seg[i]], edges[tri_table_seg[i + 1]], edges[tri_table_seg[i + 2]]));
                    ++mc_data[inds[corners[0]]].triangle_count;
                }
            }
        }
    }

    return to_return;
}

template<typename T>
inline std::shared_ptr<VV_Mesh> MarchingCubesSolver::ExtractIsosurfaceMeshFromGridWithSquaredValues(T* data, double sqrt_of_target_value)
{
    //is_edge_voxel.resize(data_size, false);

    ResetIndices();

    double target_value = sqrt_of_target_value * sqrt_of_target_value;

    auto to_return = std::make_shared<VV_Mesh>();

    size_t corners[8];

    Eigen::Vector3d positions[8];
    Eigen::Vector3d right = Eigen::Vector3d(gds.unit_length, 0, 0);
    Eigen::Vector3d up = Eigen::Vector3d(0, gds.unit_length, 0);
    Eigen::Vector3d forward = Eigen::Vector3d(0, 0, gds.unit_length);

    size_t edges[12] = { 0,0,0,0,0,0,0,0,0,0,0,0 };

    size_t visited_cells = 0;

    for (size_t x = 0; x < dim_minus_one[0]; ++x)
    {
        for (size_t y = 0; y < dim_minus_one[1]; ++y)
        {
            for (size_t z = 0; z < dim_minus_one[2]; ++z)
            {
                //(*grid_generated_triangles)[corners[0]].second = -1;

                corners[0] = z + y * span_y + x * span_x;
                corners[4] = corners[0] + span_z;
                corners[2] = corners[0] + span_y;
                corners[6] = corners[2] + span_z;
                corners[1] = corners[0] + span_x;
                corners[5] = corners[1] + span_z;
                corners[3] = corners[1] + span_y;
                corners[7] = corners[3] + span_z;

                int index = 0;

                index |= 1 * (data[corners[0]] < target_value);
                index |= 2 * (data[corners[1]] < target_value);
                index |= 8 * (data[corners[2]] < target_value);
                index |= 4 * (data[corners[3]] < target_value);
                index |= 16 * (data[corners[4]] < target_value);
                index |= 32 * (data[corners[5]] < target_value);
                index |= 128 * (data[corners[6]] < target_value);
                index |= 64 * (data[corners[7]] < target_value);

                index = 255 - index;

                if (!MarchingTables::edge_table[index])
                    continue;

                for (int i = 0; i < 8; ++i)
                {
                    if (inds[corners[i]] == SIZE_MAX)
                    {
                        CreateNewIndex(corners[i]);
                    }
                }

                ++visited_cells;

                positions[0] = Eigen::Vector3d(x, y, z) * gds.unit_length + grid_lower_bound;
                positions[4] = positions[0] + forward;
                positions[2] = positions[0] + up;
                positions[6] = positions[2] + forward;
                positions[1] = positions[0] + right;
                positions[5] = positions[1] + forward;
                positions[3] = positions[1] + up;
                positions[7] = positions[3] + forward;

                if ((MarchingTables::edge_table[index] & 1) > 0)
                    edges[0] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 0, 1, 0, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 2) > 0)
                    edges[1] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 1, 3, 1, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 4) > 0)
                    edges[2] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 2, 3, 0, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 8) > 0)
                    edges[3] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 0, 2, 1, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 16) > 0)
                    edges[4] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 4, 5, 0, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 32) > 0)
                    edges[5] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 5, 7, 1, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 64) > 0)
                    edges[6] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 6, 7, 0, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 128) > 0)
                    edges[7] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 4, 6, 1, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 256) > 0)
                    edges[8] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 0, 4, 2, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 512) > 0)
                    edges[9] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 1, 5, 2, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 1024) > 0)
                    edges[10] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 3, 7, 2, sqrt_of_target_value);
                if ((MarchingTables::edge_table[index] & 2048) > 0)
                    edges[11] = ExtractEdgeWithSqrtAdjustedValues(data, *to_return, corners, positions, 2, 6, 2, sqrt_of_target_value);

                auto tri_table_seg = MarchingTables::tri_table[index];

                mc_data[inds[corners[0]]].triangle_indices_start = to_return->vertices.indices.size();

                //Adding triangles to the mesh
                for (int i = 0; tri_table_seg[i] != -1; i += 3)
                {
                    to_return->vertices.indices.push_back(Eigen::Vector3i(edges[tri_table_seg[i]], edges[tri_table_seg[i + 1]], edges[tri_table_seg[i + 2]]));
                    ++mc_data[inds[corners[0]]].triangle_count;
                }
            }
        }
    }

    return to_return;
}

