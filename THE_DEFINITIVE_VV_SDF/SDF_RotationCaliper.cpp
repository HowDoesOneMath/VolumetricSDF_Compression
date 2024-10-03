#include "SDF_RotationCaliper.h"

void SDF_RotationCaliper::DebugDisplayPatch(VV_Mesh& mesh, std::vector<size_t>& hull, std::pair<size_t, size_t> least_box, size_t display_size)
{
    auto edges = GetHullEdges(mesh, hull);
    auto calipers = GetAntipodalPoints(*edges);

    cimg_library::CImg<unsigned char> render_texture(display_size, display_size, 1, 3);
    cimg_library::CImg<unsigned char> hull_texture(display_size, display_size, 1, 3);
    cimg_library::CImg<unsigned char> shell_texture(display_size, display_size, 1, 3);

    Eigen::Vector3<unsigned char> draw_col;
    draw_col.x() = 255;
    draw_col.y() = 255;
    draw_col.z() = 255;

    double buffer_off = 0.05;
    double line_thickness = 1.0;
    double shell_thickness = 4.0;

    Eigen::Vector2d min_bound = Eigen::Vector2d::Ones() * DBL_MAX;
    Eigen::Vector2d max_bound = -Eigen::Vector2d::Ones() * DBL_MAX;

    for (size_t i = 0; i < mesh.uvs.elements.size(); ++i)
    {
        min_bound = min_bound.cwiseMin(mesh.uvs.elements[i]);
        max_bound = max_bound.cwiseMax(mesh.uvs.elements[i]);
    }

    Eigen::Vector2d scale = Eigen::Vector2d::Ones() / ((max_bound - min_bound).maxCoeff());// *buffer_mult);

    Eigen::Vector2d offset = (max_bound + min_bound) * 0.5;// -buffer_off * Eigen::Vector2d::Ones();

    std::cout << "SCALE/OFFSET: " << scale.transpose() << " --- " << offset.transpose() << std::endl << std::endl;

    //DrawPolygon(hull_texture, mesh.uvs.elements, hull, draw_col, scale, offset, buffer_off);

    Eigen::Vector2d p0;
    Eigen::Vector2d p1;
    Eigen::Vector2d perp;

    for (size_t i = 0; i < mesh.uvs.indices.size(); ++i)
    {
        draw_col.x() = 255;
        draw_col.y() = (255.0 * i) / mesh.uvs.indices.size();
        draw_col.z() = 0;

        DrawTriangle(render_texture, mesh, i, draw_col, scale, offset, buffer_off);
    }

    for (size_t i = 0; i < hull.size(); ++i)
    {
        draw_col.x() = 0;
        draw_col.y() = (255.0 * i) / hull.size();
        draw_col.z() = 255;

        size_t p_next = (i + 1) % hull.size();

        Eigen::Vector2d dims = DrawBoundingBox(render_texture, mesh, hull[i], hull[p_next], draw_col, line_thickness, scale, offset, buffer_off);

        p0 = mesh.uvs.elements[hull[i]];
        p1 = mesh.uvs.elements[hull[p_next]];

        DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
            (p1 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(), shell_thickness, draw_col);

        std::cout << "AREA OF " << i << ":\t" << (dims.x() * dims.y() / (scale.x() * scale.y())) << std::endl;
    }

    for (size_t i = 0; i < calipers->size(); ++i)
    {
        //size_t next_p = (i + 1) % hull.size();
        p0 = mesh.uvs.elements[hull[(*calipers)[i].second.first]];
        p1 = mesh.uvs.elements[hull[(*calipers)[i].second.second]];

        draw_col.x() = 255;
        draw_col.y() = (255.0 * i) / hull.size();
        draw_col.z() = 0;
        DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
            (p1 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(), line_thickness, draw_col);

        if (i == least_box.second)
        {
            draw_col.x() = 255;
            draw_col.y() = 0;
            draw_col.z() = 255;
            DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
                (p1 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(), shell_thickness, draw_col);
        }

        if ((*calipers)[i].first == least_box.first)
        {
            size_t p_curr = (*calipers)[i].first;
            size_t p_next = (p_curr + 1) % hull.size();
            p0 = mesh.uvs.elements[hull[p_curr]];
            p1 = mesh.uvs.elements[hull[p_next]];

            draw_col.x() = 0;
            draw_col.y() = 255;
            draw_col.z() = 0;
            DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
                (p1 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(), shell_thickness, draw_col);
        }
    }

    p0 = mesh.uvs.elements[hull[0]];
    p1 = mesh.uvs.elements[hull[1]];
    perp = -Eigen::Vector2d(p1.y() - p0.y(), p0.x() - p1.x()).normalized();

    DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
        (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones() + perp * buffer_off, line_thickness, draw_col);

    p0 = mesh.uvs.elements[hull[0]];
    p1 = mesh.uvs.elements[hull[1]];
    perp = -Eigen::Vector2d(p1.y() - p0.y(), p0.x() - p1.x()).normalized();

    DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
        (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones() + perp * buffer_off, line_thickness, draw_col);


    draw_col.x() = 0;
    draw_col.y() = 255;
    draw_col.z() = 0;

    size_t least_next = (least_box.first + 1) % hull.size();

    std::cout << "\nCHOSEN: " << least_box.first << std::endl;

    DrawBoundingBox(render_texture, mesh, hull[least_box.first], hull[least_next], draw_col, line_thickness, scale, offset, buffer_off);

    std::cout << std::endl;

    //hull_texture.append(shell_texture, 'x');
    render_texture.append(hull_texture, 'x');

    cimg_library::CImgDisplay main_disp(render_texture, "Behold the shape!");

    while (!main_disp.is_closed()) {
        main_disp.wait();
    }
}

void SDF_RotationCaliper::DebugDisplayPatchLinear(VV_Mesh& mesh, std::vector<size_t>& hull, Eigen::Vector4i least_box, size_t display_size)
{
    auto edges = GetHullEdges(mesh, hull);

    cimg_library::CImg<unsigned char> render_texture(display_size, display_size, 1, 3);
    cimg_library::CImg<unsigned char> hull_texture(display_size, display_size, 1, 3);
    cimg_library::CImg<unsigned char> shell_texture(display_size, display_size, 1, 3);

    Eigen::Vector3<unsigned char> draw_col;
    draw_col.x() = 255;
    draw_col.y() = 255;
    draw_col.z() = 255;

    double buffer_off = 0.05;
    double line_thickness = 1.0;
    double shell_thickness = 4.0;

    Eigen::Vector2d min_bound = Eigen::Vector2d::Ones() * DBL_MAX;
    Eigen::Vector2d max_bound = -Eigen::Vector2d::Ones() * DBL_MAX;

    for (size_t i = 0; i < mesh.uvs.elements.size(); ++i)
    {
        min_bound = min_bound.cwiseMin(mesh.uvs.elements[i]);
        max_bound = max_bound.cwiseMax(mesh.uvs.elements[i]);
    }

    Eigen::Vector2d scale = Eigen::Vector2d::Ones() / ((max_bound - min_bound).maxCoeff());// *buffer_mult);

    Eigen::Vector2d offset = (max_bound + min_bound) * 0.5;// -buffer_off * Eigen::Vector2d::Ones();

    std::cout << "SCALE/OFFSET: " << scale.transpose() << " --- " << offset.transpose() << std::endl << std::endl;

    //DrawPolygon(hull_texture, mesh.uvs.elements, hull, draw_col, scale, offset, buffer_off);

    Eigen::Vector2d p0;
    Eigen::Vector2d p1;
    Eigen::Vector2d perp;

    for (size_t i = 0; i < mesh.uvs.indices.size(); ++i)
    {
        draw_col.x() = 255;
        draw_col.y() = (255.0 * i) / mesh.uvs.indices.size();
        draw_col.z() = 0;

        DrawTriangle(render_texture, mesh, i, draw_col, scale, offset, buffer_off);
    }

    for (size_t i = 0; i < hull.size(); ++i)
    {
        draw_col.x() = 0;
        draw_col.y() = (255.0 * i) / hull.size();
        draw_col.z() = 255;

        size_t p_next = (i + 1) % hull.size();

        Eigen::Vector2d dims = DrawBoundingBox(render_texture, mesh, hull[i], hull[p_next], draw_col, line_thickness, scale, offset, buffer_off);

        p0 = mesh.uvs.elements[hull[i]];
        p1 = mesh.uvs.elements[hull[p_next]];

        DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
            (p1 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(), shell_thickness, draw_col);

        std::cout << "AREA OF " << i << ":\t" << (dims.x() * dims.y() / (scale.x() * scale.y())) << std::endl;
    }

    draw_col.x() = 0;
    draw_col.y() = 255;
    draw_col.z() = 0;

    p0 = mesh.uvs.elements[hull[least_box[0]]];
    p1 = mesh.uvs.elements[hull[least_box[2]]];

    DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
        (p1 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(), shell_thickness, draw_col);

    draw_col.x() = 255;
    draw_col.y() = 0;
    draw_col.z() = 255;

    p0 = mesh.uvs.elements[hull[least_box[1]]];
    p1 = mesh.uvs.elements[hull[least_box[3]]];

    DrawThickLine(hull_texture, (p0 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(),
        (p1 - offset).cwiseProduct(scale) * (1.0 - 2 * buffer_off) + 0.5 * Eigen::Vector2d::Ones(), shell_thickness, draw_col);

    std::cout << "\nCHOSEN: " << least_box[0] << std::endl;

    draw_col.x() = 0;
    draw_col.y() = 255;
    draw_col.z() = 0;

    DrawBoundingBox(render_texture, mesh, hull[least_box[0]], hull[(least_box[0] + 1) % hull.size()], draw_col, line_thickness, scale, offset, buffer_off);

    std::cout << std::endl;

    render_texture.append(hull_texture, 'x');

    cimg_library::CImgDisplay main_disp(render_texture, "Behold the shape!");

    while (!main_disp.is_closed()) {
        main_disp.wait();
    }
}

std::shared_ptr<std::vector<Eigen::Vector2d>> SDF_RotationCaliper::GetHullEdges(VV_Mesh& submesh, std::vector<size_t>& hull)
{
    auto to_return = std::make_shared<std::vector<Eigen::Vector2d>>();
    to_return->resize(hull.size());

    for (size_t i = 0; i < hull.size() - 1; ++i)
    {
        (*to_return)[i] = submesh.uvs.elements[hull[i + 1]] - submesh.uvs.elements[hull[i]];
    }

    (*to_return)[to_return->size() - 1] = submesh.uvs.elements[hull[0]] - submesh.uvs.elements[hull[hull.size() - 1]];

    return to_return;
}

//This is broken, don't use it
std::shared_ptr<std::vector<std::pair<size_t, std::pair<size_t, size_t>>>> SDF_RotationCaliper::GetAntipodalPoints(std::vector<Eigen::Vector2d>& edges)
{
    auto to_return = std::make_shared<std::vector<std::pair<size_t, std::pair<size_t, size_t>>>>();

    double edge_cross = 0;
    size_t current_edge = 0;
    size_t opposite_edge = 0;

    size_t initial_opposite_edge;

    for (size_t i = 1; i < edges.size(); ++i)
    {
        edge_cross = edges[0].x() * edges[i].y() - edges[0].y() * edges[i].x();

        if (edge_cross > 0)
        {
            opposite_edge = i;
            to_return->push_back(std::make_pair((size_t)0, std::make_pair(current_edge, opposite_edge)));
            ++current_edge;

            break;
        }
    }

    initial_opposite_edge = opposite_edge;

    while (to_return->size() < edges.size())
    {
        edge_cross = edges[current_edge].x() * edges[opposite_edge].y() - edges[current_edge].y() * edges[opposite_edge].x();

        if (edge_cross > 0)
        {
            to_return->push_back(std::make_pair(current_edge, std::make_pair(current_edge, opposite_edge)));
            ++current_edge;
            current_edge *= (current_edge < edges.size());
        }
        else
        {
            to_return->push_back(std::make_pair(opposite_edge, std::make_pair(current_edge, opposite_edge)));
            ++opposite_edge;
            opposite_edge *= (opposite_edge < edges.size());
        }
    }

    return to_return;
}

//This is broken, don't use it
std::pair<Eigen::Matrix2d, std::pair<size_t, size_t>> SDF_RotationCaliper::GetOptimalRotation(VV_Mesh& submesh, std::vector<size_t>& hull)
{
    std::pair<Eigen::Matrix2d, std::pair<size_t, size_t>> to_return;

    auto edges = GetHullEdges(submesh, hull);
    auto calipers = GetAntipodalPoints(*edges);

    //std::cout << "CALIPER COUNT: " << calipers->size() << ", EDGE COUNT: " << edges->size() << std::endl;
    
    std::vector<Eigen::Vector2d> caliper_lengths;
    caliper_lengths.resize(calipers->size());

    for (size_t i = 0; i < calipers->size(); ++i)
    {
        caliper_lengths[i] = submesh.uvs.elements[hull[(*calipers)[i].second.second]] - submesh.uvs.elements[hull[(*calipers)[i].second.first]];
    }

    double greatest_dot_width = 0;
    double caliper_dot_width = 0;
    size_t perp_segment = 0;
    size_t prev_segment = 0;

    double box_height_sqr;
    double box_width_sqr;
    double box_area_sqr;
    double edge_dot_caliper;

    Eigen::Vector2d* edge_ptr;

    double least_area_sqr = DBL_MAX;
    size_t least_caliper = 0;
    size_t most_perp_caliper = 0;

    for (size_t i = 0; i < calipers->size(); ++i)
    {
        edge_ptr = &(*edges)[(*calipers)[i].first];

        edge_dot_caliper = caliper_lengths[i].dot(*edge_ptr);
        box_height_sqr = caliper_lengths[i].squaredNorm() - edge_dot_caliper * edge_dot_caliper / edge_ptr->squaredNorm();

        if (i > 0)
        {
            caliper_dot_width = edge_ptr->dot(caliper_lengths[perp_segment]);
            caliper_dot_width *= caliper_dot_width;
            caliper_dot_width /= edge_ptr->squaredNorm();
        }

        //Find the most perpendicular caliper. We save time by keeping track of the previous caliper, as they are ordered in a spiral.
        do
        {
            prev_segment = perp_segment;
            ++perp_segment;
            perp_segment *= (perp_segment < calipers->size());

            greatest_dot_width = caliper_dot_width;

            caliper_dot_width = edge_ptr->dot(caliper_lengths[perp_segment]);
            caliper_dot_width *= caliper_dot_width;
            caliper_dot_width /= edge_ptr->squaredNorm();

            //When the next caliper is less long than the previous, we know it's hit maximum perpendicularity.
            //Use the previous caliper to measure the box width.
        } while (caliper_dot_width > greatest_dot_width);

        perp_segment = prev_segment;

        edge_dot_caliper = caliper_lengths[perp_segment].dot(*edge_ptr);
        box_width_sqr = edge_dot_caliper * edge_dot_caliper / edge_ptr->squaredNorm();

        box_area_sqr = box_width_sqr * box_height_sqr;

        std::cout << "Calculated area for caliper " << i << " (edge " << (*calipers)[i].first << ", connecting " 
            << (*calipers)[i].second.first << " - " << (*calipers)[i].second.second << "):\t"
            << sqrt(box_area_sqr) << " (" << sqrt(least_area_sqr) << ")\t(" << perp_segment << ")" << std::endl;

        if (box_area_sqr < least_area_sqr)
        {
            least_area_sqr = box_area_sqr;
            least_caliper = i;
            most_perp_caliper = perp_segment;
        }
    }

    size_t least_edge = (*calipers)[least_caliper].first;
    edge_ptr = &(*edges)[least_edge];
    Eigen::Vector2d chosen_edge = edge_ptr->normalized();

    std::cout << "CHOSEN CALIPER: " << least_caliper << " (with edge " << least_edge << ")" << std::endl;
    std::cout << "CHOSEN PERP: " << most_perp_caliper << std::endl;

    to_return.second.first = least_edge;
    to_return.second.second = most_perp_caliper;
    to_return.first(0, 0) = chosen_edge.x();
    to_return.first(0, 1) = chosen_edge.y();
    to_return.first(1, 0) = -chosen_edge.y();
    to_return.first(1, 1) = chosen_edge.x();

    return to_return;
}

std::pair<Eigen::Matrix2d, Eigen::Vector4i> SDF_RotationCaliper::GetOptimalRotationLinear(VV_Mesh& submesh, std::vector<size_t>& hull)
{
    std::pair<Eigen::Matrix2d, Eigen::Vector4i> to_return;

    auto edges = GetHullEdges(submesh, hull);

    size_t initial_edge = 0;
    size_t left_edge = 0;
    size_t opposite_edge = 0;
    size_t right_edge = 0;

    while ((*edges)[initial_edge].dot((*edges)[left_edge]) > 0)
    {
        ++left_edge;
        left_edge *= (left_edge < edges->size());
    }

    opposite_edge = left_edge;

    while (((*edges)[initial_edge].x() * (*edges)[opposite_edge].y() - (*edges)[initial_edge].y() * (*edges)[opposite_edge].x()) < 0)
    {
        ++opposite_edge;
        opposite_edge *= (opposite_edge < edges->size());
    }

    right_edge = opposite_edge;

    while ((*edges)[initial_edge].dot((*edges)[right_edge]) < 0)
    {
        ++right_edge;
        right_edge *= (right_edge < edges->size());
    }

    Eigen::Vector2d perpendicular_axis;
    Eigen::Vector2d parallel_axis;

    double edge_dot_perpendicular;
    double edge_dot_parallel;

    double least_sqr_area = DBL_MAX;
    size_t least_index_initial = 0;
    size_t least_index_left = 0;
    size_t least_index_opposite = 0;
    size_t least_index_right = 0;

    double test_sqr_area;

    double edge_sqr_length;

    for (; initial_edge < edges->size(); ++initial_edge)
    {
        while ((*edges)[initial_edge].dot((*edges)[left_edge]) > 0)
        {
            ++left_edge;
            left_edge *= (left_edge < edges->size());
        }

        while (((*edges)[initial_edge].x() * (*edges)[opposite_edge].y() - (*edges)[initial_edge].y() * (*edges)[opposite_edge].x()) < 0)
        {
            ++opposite_edge;
            opposite_edge *= (opposite_edge < edges->size());
        }

        while ((*edges)[initial_edge].dot((*edges)[right_edge]) < 0)
        {
            ++right_edge;
            right_edge *= (right_edge < edges->size());
        }

        parallel_axis = submesh.uvs.elements[hull[left_edge]] - submesh.uvs.elements[hull[right_edge]];
        perpendicular_axis = submesh.uvs.elements[hull[opposite_edge]] - submesh.uvs.elements[hull[initial_edge]];

        edge_sqr_length = (*edges)[initial_edge].squaredNorm();

        edge_dot_parallel = parallel_axis.dot((*edges)[initial_edge]);
        edge_dot_perpendicular = perpendicular_axis.dot((*edges)[initial_edge]);

        test_sqr_area = (edge_dot_parallel * edge_dot_parallel / edge_sqr_length)
            * (perpendicular_axis.squaredNorm() - edge_dot_perpendicular * edge_dot_perpendicular / edge_sqr_length);

        if (test_sqr_area < least_sqr_area)
        {
            least_index_initial = initial_edge;
            least_index_left = left_edge;
            least_index_opposite = opposite_edge;
            least_index_right = right_edge;

            least_sqr_area = test_sqr_area;
        }
    }

    Eigen::Vector2d least_edge = (*edges)[least_index_initial].normalized();

    to_return.second[0] = least_index_initial;
    to_return.second[1] = least_index_left;
    to_return.second[2] = least_index_opposite;
    to_return.second[3] = least_index_right;

    to_return.first(0, 0) = least_edge.x();
    to_return.first(0, 1) = least_edge.y();
    to_return.first(1, 0) = -least_edge.y();
    to_return.first(1, 1) = least_edge.x();

    return to_return;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> SDF_RotationCaliper::RotateUVs(VV_Mesh& mc_mesh)
{
    std::pair<Eigen::Vector2d, Eigen::Vector2d> to_return;
    to_return.first = Eigen::Vector2d(DBL_MAX, DBL_MAX);
    to_return.second = Eigen::Vector2d(-DBL_MAX, -DBL_MAX);

    auto hull = chc.QuickHullOfUVs(mc_mesh);

    //auto optimal_rot = GetOptimalRotation(mc_mesh, *hull);

    auto optimal_rot = GetOptimalRotationLinear(mc_mesh, *hull);

    for (size_t i = 0; i < mc_mesh.uvs.elements.size(); ++i)
    {
        mc_mesh.uvs.elements[i] = optimal_rot.first * mc_mesh.uvs.elements[i];

        to_return.first = to_return.first.cwiseMin(mc_mesh.uvs.elements[i]);
        to_return.second = to_return.second.cwiseMax(mc_mesh.uvs.elements[i]);
    }

    //DebugDisplayPatch(mc_mesh, *hull, optimal_rot.second);
    //DebugDisplayPatchLinear(mc_mesh, *hull, optimal_rot.second);

    return to_return;
}

std::shared_ptr<std::vector<std::pair<Eigen::Vector3d, std::vector<size_t>>>> SDF_RotationCaliper::GetGroupsByNormal(VV_Mesh &mc_mesh, double minimum_normal_similarity)
{
    auto to_return = std::make_shared<std::vector<std::pair<Eigen::Vector3d, std::vector<size_t>>>>();

    auto normal_parts = mp.CreateNormalPartitions(mc_mesh, minimum_normal_similarity);

    for (size_t i = 0; i < normal_parts->size(); ++i)
    {
        if ((*normal_parts)[i].second.size() <= 0)
        {
            continue;
        }

        auto mesh_section_by_normal = mc_mesh.ExtractSubmesh((*normal_parts)[i].second);

        mp.CreateUnionFindPartitions(*mesh_section_by_normal);

        for (size_t j = 0; j < mp.partitions.size(); ++j)
        {
            to_return->push_back(std::make_pair((*normal_parts)[i].first, std::vector<size_t>()));

            for (size_t k = 0; k < mp.partitions[j].triangle_indices.size(); ++k)
            {
                //The partition indices are relative the sub-mesh acquired from the normals.
                //This means they can be used as an index into the normal array, to find the index relative to the original mesh.
                to_return->back().second.push_back(
                    (*normal_parts)[i].second[
                        mp.partitions[j].triangle_indices[k]
                    ]
                );
            }
        }

        mp.ClearPartitions();
    }

    return to_return;
}

void SDF_RotationCaliper::ConstructRotationCaliperAtlas(VV_Mesh& mc_mesh, Eigen::Vector2d spot_start, Eigen::Vector2d spot_dims, 
    double minimum_normal_similarity, double pad_amount)
{	
    //Divide the SDF chunk by normal and locality
    auto groups_by_normal = GetGroupsByNormal(mc_mesh, minimum_normal_similarity);

    std::vector<std::pair<std::vector<size_t>, std::vector<size_t>>> groups_indices_elements;
    groups_indices_elements.resize(groups_by_normal->size());
    
    for (size_t i = 0; i < groups_indices_elements.size(); ++i)
    {
        groups_indices_elements[i].first = (*groups_by_normal)[i].second;
    }
    
    Eigen::Vector3i* tri;
    Eigen::Vector3d* v0;
    Eigen::Vector3d* v1;
    Eigen::Vector3d* v2;
    
    Eigen::Vector3d rotated;
    
    Eigen::Vector2d avg_texcoord;

    Eigen::Vector2d pad_2d = pad_amount * Eigen::Vector2d::Ones();
    
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> bounding_boxes;
    bounding_boxes.resize(groups_indices_elements.size());
    
    mc_mesh.uvs.indices.resize(mc_mesh.vertices.indices.size());
    
    //std::cout << "Group count: " << groups_indices_elements.size() << std::endl;

    for (size_t i = 0; i < groups_indices_elements.size(); ++i)
    {
        if (groups_indices_elements[i].first.size() <= 0)
        {
            continue;
        }

        auto atlas_section = mc_mesh.ExtractSubmesh(groups_indices_elements[i].first, VV_Mesh::VV_Mesh_Ignore_Flags::IGNORE_UVS);
    
        Eigen::Vector3d avg_normal = Eigen::Vector3d::Zero();
        
        for (size_t j = 0; j < atlas_section->vertices.indices.size(); ++j)
        {
            tri = &atlas_section->vertices.indices[j];
            v0 = &atlas_section->vertices.elements[tri->x()];
            v1 = &atlas_section->vertices.elements[tri->y()];
            v2 = &atlas_section->vertices.elements[tri->z()];
        
            avg_normal += CrossProduct(*v1 - *v0, *v2 - *v0);
        }
        
        //avg_normal /= atlas_section->vertices.indices.size();
        
        Eigen::Quaterniond to_normal = Eigen::Quaterniond::FromTwoVectors(avg_normal.normalized(), Eigen::Vector3d(0, 0, 1));
        
        atlas_section->uvs.indices = atlas_section->vertices.indices;
        atlas_section->uvs.elements.resize(atlas_section->vertices.elements.size());
        
        Eigen::Vector2d cwise_buff;

        bounding_boxes[i].first = Eigen::Vector2d(DBL_MAX, DBL_MAX);
        bounding_boxes[i].second = Eigen::Vector2d(-DBL_MAX, -DBL_MAX);

        for (size_t j = 0; j < atlas_section->uvs.elements.size(); ++j)
        {
            rotated = to_normal * atlas_section->vertices.elements[j];
            atlas_section->uvs.elements[j] = Eigen::Vector2d(rotated.x(), rotated.y());

            //bounding_boxes[i].first = bounding_boxes[i].first.cwiseMin(atlas_section->uvs.elements[j]);
            //bounding_boxes[i].second = bounding_boxes[i].second.cwiseMax(atlas_section->uvs.elements[j]);
        }

        bounding_boxes[i] = RotateUVs(*atlas_section);

        bounding_boxes[i].first -= pad_2d;
        bounding_boxes[i].second += pad_2d;

        size_t last_uv_count = mc_mesh.uvs.elements.size();
        Eigen::Vector3i last_uv_triplet = Eigen::Vector3i(last_uv_count, last_uv_count, last_uv_count);
        mc_mesh.uvs.elements.resize(last_uv_count + atlas_section->uvs.elements.size());
        groups_indices_elements[i].second.resize(atlas_section->uvs.elements.size());

        for (size_t j = 0; j < atlas_section->uvs.elements.size(); ++j)
        {
            mc_mesh.uvs.elements[j + last_uv_count] = atlas_section->uvs.elements[j];
            groups_indices_elements[i].second[j] = j + last_uv_count;
        }
        
        for (size_t j = 0; j < atlas_section->uvs.indices.size(); ++j)
        {
            mc_mesh.uvs.indices[groups_indices_elements[i].first[j]] = atlas_section->uvs.indices[j] + last_uv_triplet;
        }
    }

    auto instructions = rp.GetMergeOperations(bounding_boxes);
    size_t instruction_index = instructions->size();
    Eigen::Vector2i box_indices;
    Eigen::Vector2d shift;
    Eigen::Vector2d box_size;
    std::vector<Eigen::Vector2d> box_deltas;
    box_deltas.resize(bounding_boxes.size(), Eigen::Vector2d::Zero());

    while (instruction_index > 0)
    {
        --instruction_index;

        box_indices = (*instructions)[instruction_index].second;
        shift = (*instructions)[instruction_index].first;

        box_deltas[box_indices[0]] = bounding_boxes[box_indices[1]].first + shift - bounding_boxes[box_indices[0]].first;

        box_size = bounding_boxes[box_indices[0]].second - bounding_boxes[box_indices[0]].first;
        bounding_boxes[box_indices[0]].first = bounding_boxes[box_indices[1]].first + shift;
        bounding_boxes[box_indices[0]].second = bounding_boxes[box_indices[0]].first + box_size;
    }

    std::pair<Eigen::Vector2d, Eigen::Vector2d> encompassing_bb;
    encompassing_bb.first.x() = DBL_MAX;
    encompassing_bb.first.y() = DBL_MAX;
    encompassing_bb.second.x() = -DBL_MAX;
    encompassing_bb.second.y() = -DBL_MAX;

    for (size_t i = 0; i < bounding_boxes.size(); ++i)
    {
        encompassing_bb.first = encompassing_bb.first.cwiseMin(bounding_boxes[i].first);
        encompassing_bb.second = encompassing_bb.second.cwiseMax(bounding_boxes[i].second);

        for (size_t j = 0; j < groups_indices_elements[i].second.size(); ++j)
        {
            mc_mesh.uvs.elements[groups_indices_elements[i].second[j]] += box_deltas[i];
        }
    }

    Eigen::Vector2d final_spans = encompassing_bb.second - encompassing_bb.first;
    Eigen::Vector2d inverse_spans = Eigen::Vector2d(1.0 / final_spans.x(), 1.0 / final_spans.y());

    for (size_t i = 0; i < mc_mesh.uvs.elements.size(); ++i)
    {
        mc_mesh.uvs.elements[i] = (mc_mesh.uvs.elements[i] - encompassing_bb.first).cwiseProduct(inverse_spans).cwiseProduct(spot_dims) + spot_start;
    }

    //if (bounding_boxes.size() > 1)
    //{
    //    Eigen::Vector2d bb_size;
    //
    //    for (size_t i = 0; i < bounding_boxes.size(); ++i)
    //    {
    //        bb_size = bounding_boxes[i].second - bounding_boxes[i].first;
    //        std::cout << "\t" << i << ": " << bb_size.transpose() << "\t\t" << bounding_boxes[i].first.transpose() << ", " << bounding_boxes[i].second.transpose() << "\n";
    //    }
    //
    //    bb_size = encompassing_bb.second - encompassing_bb.first;
    //
    //    std::cout << "TOT: " << bb_size.transpose() << "\t\t" << encompassing_bb.first.transpose() << ", " << encompassing_bb.second.transpose() << "\n";
    //
    //    std::cout << "Group count: " << bounding_boxes.size() << std::endl;
    //}
}
