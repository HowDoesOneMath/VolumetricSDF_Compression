#include "ConvexHullCreator.h"


std::shared_ptr<std::vector<size_t>> ConvexHullCreator::GetHullFromPoints(VV_Mesh& mesh,
    std::unordered_set<size_t>& target_points, size_t axis_point_0, size_t axis_point_1)
{
    auto to_return = std::make_shared<std::vector<size_t>>();

    Eigen::Vector2d* a_0 = &mesh.uvs.elements[axis_point_0];
    Eigen::Vector2d* a_1 = &mesh.uvs.elements[axis_point_1];

    auto exterior_points = GetExteriorPoints(mesh, target_points, a_0, a_1);

    //std::cout << "Exterior points for " << axis_point_0 << ", " << axis_point_1 << ": " << exterior_points->first.size() << std::endl;

    //If no points were above the line, we've found a bounding edge and no further calculation needs to happen.
    if (exterior_points->second == SIZE_MAX)
    {
        //std::cout << "EDGE FOUND: " << axis_point_0 << ", " << axis_point_1 << std::endl;
        return to_return;
    }

    std::shared_ptr<std::vector<size_t>> new_hull;

    //Avoid testing a future axis point if it's a new axis point, as it could lead to errors.
    exterior_points->first.erase(exterior_points->second);

    //std::cout << "NEW TRIANGLE: " << axis_point_0 << ", " << axis_point_1 << ", " << exterior_points->second << std::endl;

    //Form a triangle between the old axis and the new exterior point. The new triangle faces become preceeding and proceeding axes.
    //                    * edge point
    //                   /|\
    // preceeding axis  / | \  proceeding axis
    //                 /  |  \
    //                A0 --- A1
    //                   ^ old axis

    //To deal with an edge case of points being along a straight line, points are divided from the left/right side of the edge point (along the axis)
    //The new points become checking points for the preceeding/proceeding axes
    auto point_split = SplitPointsPerpendicularToAxis(mesh, exterior_points->first, *a_1 - *a_0, exterior_points->second);
    
    //std::cout << "Left group: " << point_split->first.size() << ", Right group: " << point_split->second.size() << std::endl;

    //Add all of the preceeding edge points to the new representation.
    new_hull = GetHullFromPoints(mesh, point_split->first, axis_point_0, exterior_points->second);
    if (new_hull->size() > 0)
    {
        to_return->insert(to_return->end(), new_hull->begin(), new_hull->end());
    }

    //Add the furthest edge point as the middle between the preceeding and proceeding sets.
    to_return->push_back(exterior_points->second);

    //Add the proceeding edge points finally
    new_hull = GetHullFromPoints(mesh, point_split->second, exterior_points->second, axis_point_1);
    if (new_hull->size() > 0)
    {
        to_return->insert(to_return->end(), new_hull->begin(), new_hull->end());
    }

    return to_return;
}

std::shared_ptr<std::pair<std::unordered_set<size_t>, std::unordered_set<size_t>>> ConvexHullCreator::SplitPointsPerpendicularToAxis(VV_Mesh& mesh, 
    std::unordered_set<size_t>& target_points, Eigen::Vector2d axis, size_t separating_index)
{
    auto to_return = std::make_shared<std::pair<std::unordered_set<size_t>, std::unordered_set<size_t>>>();

    Eigen::Vector2d* test_point;

    Eigen::Vector2d* separating_point = &mesh.uvs.elements[separating_index];
    Eigen::Vector2d p_sep;
    double axis_side;

    //Split points according to if they're on the left or right side of the axis, as determined by the location of the separating point.
    for (auto& ind : target_points)
    {
        test_point = &mesh.uvs.elements[ind];

        p_sep = *test_point - *separating_point;

        //Cross product between the delta of the point in question to the separating point, and a vector perpendicular to the axis.
        //In other words, a dot product.
        axis_side = axis.y() * p_sep.y() + axis.x() * p_sep.x();

        if (axis_side < 0)
        {
            //Preceeding points
            to_return->first.insert(ind);
        }
        else
        {
            //Proceeding points
            to_return->second.insert(ind);
        }
    }

    return to_return;
}

std::shared_ptr<std::pair<std::unordered_set<size_t>, size_t>> ConvexHullCreator::GetExteriorPoints(VV_Mesh& mesh,
    std::unordered_set<size_t>& target_points, Eigen::Vector2d* a_0, Eigen::Vector2d* a_1)
{
    auto to_return = std::make_shared<std::pair<std::unordered_set<size_t>, size_t>>();
    to_return->second = SIZE_MAX;

    Eigen::Vector2d* test_point;
    Eigen::Vector2d axis = *a_1 - *a_0;
    Eigen::Vector2d p_a0;

    double closest_distance = -DBL_MAX;
    double pa0_dot_axis;
    double distance_to_axis;
    double axis_side;

    //Deterine if any given point is on the exterior of the axis, and if so record the furthest point.
    for (auto& ind : target_points)
    {
        test_point = &mesh.uvs.elements[ind];

        p_a0 = *test_point - *a_0;

        axis_side = axis.x() * p_a0.y() - axis.y() * p_a0.x();

        //If the point is below the axis, then it must reside inside the hull and can be safely discarded.
        if (axis_side < 0)
        {
            continue;
        }

        to_return->first.insert(ind);

        //Quick distance calculation from the point to the axis.
        //A naive line calculation is used (i.e. assuming the line is infinitely long), as no exterior points may extend past the line
        pa0_dot_axis = p_a0.dot(axis);
        distance_to_axis = p_a0.squaredNorm() - (pa0_dot_axis * pa0_dot_axis) / axis.squaredNorm();

        //std::cout << "\tAXIS DISTANCE OF " << ind << ": " << distance_to_axis << std::endl;

        if (distance_to_axis > closest_distance)
        {
            closest_distance = distance_to_axis;
            to_return->second = ind;
        }
    }

    return to_return;
}

std::shared_ptr<std::vector<size_t>> ConvexHullCreator::QuickHullOfUVs(VV_Mesh& mesh)
{
    auto to_return = std::make_shared<std::vector<size_t>>();

    if (mesh.uvs.elements.size() <= 0)
    {
        std::cout << "ERROR: no mesh uvs present! " << std::endl;
        return to_return;
    }

    size_t min_point_x = 0;
    size_t max_point_x = 0;

    Eigen::Vector2d* uv_point;
    Eigen::Vector2d* uv_min = &mesh.uvs.elements[0];
    Eigen::Vector2d* uv_max = &mesh.uvs.elements[0];

    std::unordered_set<size_t> to_test;
    to_test.insert(0);

    //Formulate a separating axes from the minimum and maximum points on the X axis
    for (size_t i = 1; i < mesh.uvs.elements.size(); ++i)
    {
        uv_point = &mesh.uvs.elements[i];
        to_test.insert(i);

        //Get lowest/highest X coords. Use Y coord for ties in X value.
        if (uv_point->x() < uv_min->x())
        {
            min_point_x = i;
            uv_min = uv_point;
        }
        else if (uv_point->x() == uv_min->x())
        {
            if (uv_point->y() < uv_min->y())
            {
                min_point_x = i;
                uv_min = uv_point;
            }
        }

        if (uv_point->x() > uv_max->x())
        {
            max_point_x = i;
            uv_max = uv_point;
        }
        else if (uv_point->x() == uv_max->x())
        {
            if (uv_point->y() > uv_max->y())
            {
                max_point_x = i;
                uv_max = uv_point;
            }
        }
    }

    //std::cout << "Min/Max: " << min_point_x << ", " << max_point_x << std::endl;

    //Erase the axis points from the test group, as they must be part of the final representation.
    to_test.erase(min_point_x);
    to_test.erase(max_point_x);

    std::shared_ptr<std::vector<size_t>> new_hull;

    //By reversing the hull direction, we ensure that despite only the top points 
    // being recorded in GetHullFromPoints, both sides of the separating axis are accounted for.
    to_return->push_back(min_point_x);
    new_hull = GetHullFromPoints(mesh, to_test, min_point_x, max_point_x);
    to_return->insert(to_return->end(), new_hull->begin(), new_hull->end());

    to_return->push_back(max_point_x);
    new_hull = GetHullFromPoints(mesh, to_test, max_point_x, min_point_x);
    to_return->insert(to_return->end(), new_hull->begin(), new_hull->end());

    return to_return;
}
