#include "TextureRemapper.h"

#include <chrono>

void TextureRemapper::CopyPoint(cimg_library::CImg<unsigned char>& original_texture, cimg_library::CImg<unsigned char>& remapped_texture, Eigen::Vector2d& original_uv, int remap_w, int remap_h)
{
    Eigen::Vector2d pixel_coord(original_texture.width() * original_uv.x(), original_texture.height() * (1.0 - original_uv.y()));

    //std::cout << pixel_coord.transpose() << std::endl;

    double t_x = pixel_coord.x();
    t_x = t_x - floor(t_x);
    double one_minus_t_x = 1.0 - t_x;

    double t_y = pixel_coord.y();
    t_y = t_y - floor(t_y);
    double one_minus_t_y = 1.0 - t_y;

    int lesser_x = std::clamp((int)floor(pixel_coord.x()), 0, original_texture.width() - 1);
    int greater_x = std::min(lesser_x + 1, original_texture.width() - 1);

    int lesser_y = std::clamp((int)floor(pixel_coord.y()), 0, original_texture.height() - 1);
    int greater_y = std::min(lesser_y + 1, original_texture.height() - 1);

    //double r_00 = original_texture(lesser_x, lesser_y, 0, 0) * one_minus_t_x * one_minus_t_y;
    //double g_00 = original_texture(lesser_x, lesser_y, 0, 1) * one_minus_t_x * one_minus_t_y;
    //double b_00 = original_texture(lesser_x, lesser_y, 0, 2) * one_minus_t_x * one_minus_t_y;
    //
    //double r_01 = original_texture(lesser_x, greater_y, 0, 0) * one_minus_t_x * t_y;
    //double g_01 = original_texture(lesser_x, greater_y, 0, 1) * one_minus_t_x * t_y;
    //double b_01 = original_texture(lesser_x, greater_y, 0, 2) * one_minus_t_x * t_y;
    //
    //double r_10 = original_texture(greater_x, lesser_y, 0, 0) * t_x * one_minus_t_y;
    //double g_10 = original_texture(greater_x, lesser_y, 0, 1) * t_x * one_minus_t_y;
    //double b_10 = original_texture(greater_x, lesser_y, 0, 2) * t_x * one_minus_t_y;
    //
    //double r_11 = original_texture(greater_x, greater_y, 0, 0) * t_x * t_y;
    //double g_11 = original_texture(greater_x, greater_y, 0, 1) * t_x * t_y;
    //double b_11 = original_texture(greater_x, greater_y, 0, 2) * t_x * t_y;

    Eigen::Vector3d rgb_00 = GetPixelDouble(original_texture, lesser_x, lesser_y) * one_minus_t_x * one_minus_t_y;
    Eigen::Vector3d rgb_01 = GetPixelDouble(original_texture, lesser_x, greater_y) * one_minus_t_x * t_y;
    Eigen::Vector3d rgb_10 = GetPixelDouble(original_texture, greater_x, lesser_y) * t_x * one_minus_t_y;
    Eigen::Vector3d rgb_11 = GetPixelDouble(original_texture, greater_x, greater_y) * t_x * t_y;

    //std::cout << original_uv.transpose() << ": " << Eigen::Vector3d(r_00 + r_01 + r_10 + r_11, g_00 + g_01 + g_10 + g_11, b_00 + b_01 + b_10 + b_11).transpose() << std::endl;

    SetPixelDouble(remapped_texture, remap_w, remap_h, rgb_00 + rgb_01 + rgb_10 + rgb_11);

    //remapped_texture(remap_w, remap_h, 0, 0) = r_00 + r_01 + r_10 + r_11;
    //remapped_texture(remap_w, remap_h, 0, 1) = g_00 + g_01 + g_10 + g_11;
    //remapped_texture(remap_w, remap_h, 0, 2) = b_00 + b_01 + b_10 + b_11;
}

Eigen::Vector3i TextureRemapper::GetPixel(cimg_library::CImg<unsigned char>& tex, int x, int y)
{
    return Eigen::Vector3i(tex(x, y, 0, 0), tex(x, y, 0, 1), tex(x, y, 0, 2));
}

Eigen::Vector3d TextureRemapper::GetPixelDouble(cimg_library::CImg<unsigned char>& tex, int x, int y)
{
    return Eigen::Vector3d(tex(x, y, 0, 0), tex(x, y, 0, 1), tex(x, y, 0, 2));
}

void TextureRemapper::SetPixel(cimg_library::CImg<unsigned char>& tex, int x, int y, Eigen::Vector3i value)
{
    tex(x, y, 0, 0) = value.x();
    tex(x, y, 0, 1) = value.y();
    tex(x, y, 0, 2) = value.z();
}

void TextureRemapper::SetPixelDouble(cimg_library::CImg<unsigned char>& tex, int x, int y, Eigen::Vector3d value)
{
    tex(x, y, 0, 0) = std::floor(value.x() + 0.5);
    tex(x, y, 0, 1) = std::floor(value.y() + 0.5);
    tex(x, y, 0, 2) = std::floor(value.z() + 0.5);
}

bool TextureRemapper::LazyUV_Generation(VV_Mesh& to_apply_uvs, Eigen::Vector2i uv_divs, double uv_buffer)
{
    to_apply_uvs.uvs.indices.resize(to_apply_uvs.vertices.indices.size());
    to_apply_uvs.uvs.elements.resize(to_apply_uvs.uvs.indices.size() * 3);

    size_t current_triangle = 0;
    //size_t uv_loc[2];

    size_t x_coord = 0;
    size_t y_coord = 0;

    Eigen::Vector2d area_size = Eigen::Vector2d(1.0 / uv_divs.x(), 1.0 / uv_divs.y());
    Eigen::Vector2d area_upper_bound;
    Eigen::Vector2d area_lower_bound;

    size_t current_indices = 3;

    for (size_t i = 1; i < to_apply_uvs.vertices.indices.size(); i += 2, current_indices += 6)
    {
        ++x_coord;
        if (x_coord >= uv_divs.x())
        {
            x_coord = 0;
            ++y_coord;
        }

        area_upper_bound.x() = area_size.x() * ((double)(x_coord + 1) - uv_buffer);
        area_upper_bound.y() = area_size.y() * ((double)(y_coord + 1) - 2 * uv_buffer);

        area_lower_bound.x() = area_size.x() * ((double)(x_coord)+2 * uv_buffer);
        area_lower_bound.y() = area_size.y() * ((double)(y_coord)+uv_buffer);

        to_apply_uvs.uvs.indices[i] = Eigen::Vector3i(current_indices, current_indices + 1, current_indices + 2);

        to_apply_uvs.uvs.elements[current_indices] = area_upper_bound;
        to_apply_uvs.uvs.elements[current_indices + 1] = area_lower_bound;
        to_apply_uvs.uvs.elements[current_indices + 2] = Eigen::Vector2d(area_upper_bound.x(), area_lower_bound.y());
    }

    x_coord = 0;
    y_coord = 0;
    current_indices = 0;

    for (size_t i = 0; i < to_apply_uvs.vertices.indices.size(); i += 2, current_indices += 6)
    {
        ++x_coord;
        if (x_coord >= uv_divs.x())
        {
            x_coord = 0;
            ++y_coord;
        }

        area_upper_bound.x() = area_size.x() * ((double)(x_coord + 1) - 2 * uv_buffer);
        area_upper_bound.y() = area_size.y() * ((double)(y_coord + 1) - uv_buffer);

        area_lower_bound.x() = area_size.x() * ((double)(x_coord)+uv_buffer);
        area_lower_bound.y() = area_size.y() * ((double)(y_coord)+2 * uv_buffer);

        to_apply_uvs.uvs.indices[i] = Eigen::Vector3i(current_indices, current_indices + 1, current_indices + 2);

        to_apply_uvs.uvs.elements[current_indices] = area_upper_bound;
        to_apply_uvs.uvs.elements[current_indices + 1] = area_lower_bound;
        to_apply_uvs.uvs.elements[current_indices + 2] = Eigen::Vector2d(area_lower_bound.x(), area_upper_bound.y());
    }

    if (y_coord >= uv_divs.y())
    {
        std::cout << "Insufficient uv divisions for uv coordinates! ("
            << current_triangle << " tris -> " << uv_divs.x() << " x " << uv_divs.y() << " sections)" << std::endl;
        return false;
    }

    return true;

}

void TextureRemapper::PadTexture(cimg_library::CImg<unsigned char>& tex, std::vector<bool>& occupancy, int loops)
{
    //std::cout << "Pad count: " << loops << std::endl;

    if (loops <= 0)
    {
        return;
    }

    

    std::vector<bool> occupancy_buffer;
    occupancy_buffer.resize(occupancy.size(), false);

    int occ_loc;

    int occ_loc_n; //north
    int occ_loc_e; //east
    int occ_loc_s; //south
    int occ_loc_w; //west

    int div_count;
    double divisors[5] = { 
        0.0, 
        1.0, 
        0.5, 
        1.0 / 3.0, 
        0.25 
    };

    Eigen::Vector3d values;
    Eigen::Vector3d maxCharAsDouble = Eigen::Vector3d::Ones() * 255.0;

    for (int i = 0; i < loops; ++i)
    {
        occ_loc = 0;

        occ_loc_n = 1; 
        occ_loc_e = tex.height(); 
        occ_loc_s = -1; 
        occ_loc_w = -tex.height(); 

        for (int w = 0; w < tex.width(); ++w)
        {
            for (int h = 0; h < tex.height(); ++h, ++occ_loc, ++occ_loc_n, ++occ_loc_e, ++occ_loc_s, ++occ_loc_w)
            {
                if (occupancy[occ_loc])
                {
                    continue;
                }

                values = Eigen::Vector3d::Zero();
                div_count = 0;

                if ((h < tex.height() - 1) && occupancy[occ_loc_n])
                {
                    ++div_count;
                    values += GetPixelDouble(tex, w, h + 1);
                }
                if ((w < tex.width() - 1) && occupancy[occ_loc_e])
                {
                    ++div_count;
                    values += GetPixelDouble(tex, w + 1, h);
                }
                if ((h > 0) && occupancy[occ_loc_s])
                {
                    ++div_count;
                    values += GetPixelDouble(tex, w, h - 1);
                }
                if ((w > 0) && occupancy[occ_loc_w])
                {
                    ++div_count;
                    values += GetPixelDouble(tex, w - 1, h);
                }

                if (div_count <= 0)
                {
                    continue;
                }

                occupancy_buffer[occ_loc] = true;
                //++written;

                values *= divisors[div_count];
                values = maxCharAsDouble.cwiseMin(values.cwiseMax(Eigen::Vector3d::Zero()));
                
                SetPixelDouble(tex, w, h, values);
            }
        }

        for (int j = 0; j < occupancy_buffer.size(); ++j)
        {
            occupancy[j] = (occupancy[j] || occupancy_buffer[j]);
            occupancy_buffer[j] = false;
        }
    }
}

cimg_library::CImg<double> TextureRemapper::GetGaussianKernel(int width, double scale, bool normalized)
{
    cimg_library::CImg<double> to_return;
    cimg_library::CImg<double> gaussian_spread;

    to_return.assign(width, width, 1, 1, 1.0);
    gaussian_spread.assign(1, width, 1, 1);

    double inverse_scale = 1.0 / scale;

    double center = (width - 1.0) * 0.5;

    double exp_val;

    for (int w = 0; w < width; ++w)
    {
        exp_val = (w - center) * inverse_scale;
        gaussian_spread(0, w, 0, 0) = exp(-exp_val * exp_val);
    }

    to_return.mul(gaussian_spread);
    to_return.transpose();
    to_return.mul(gaussian_spread);

    if (normalized)
    {
        to_return /= to_return.sum();
    }

    return to_return;
}

void TextureRemapper::PadTextureWithPushPull(cimg_library::CImg<unsigned char>& tex, std::vector<bool>& occupancy,
    cimg_library::CImg<double> pull_kernel, cimg_library::CImg<double> push_kernel)
{
    size_t img_w = tex.width();
    size_t img_h = tex.height();

    if ((img_w == 1) && (img_h == 1))
    {
        return;
    }

    std::vector<std::pair<cimg_library::CImg<double>, cimg_library::CImg<double>>> image_pyramid;

    cimg_library::CImg<double> rgb_stream(img_w, img_h, 1, 3);
    cimg_library::CImg<double> unclamped_occupancy;
    cimg_library::CImg<double> occupancy_stream(img_w, img_h, 1, 1);

    rgb_stream = tex / 255.0;

#pragma omp parallel for
    for (int w = 0; w < img_w; ++w)
    {
        for (int h = 0; h < img_h; ++h)
        {
            //rgb_stream(w, h, 0, 0) = tex(w, h, 0, 0) / 255.0;
            //rgb_stream(w, h, 0, 1) = tex(w, h, 0, 1) / 255.0;
            //rgb_stream(w, h, 0, 2) = tex(w, h, 0, 2) / 255.0;
            occupancy_stream(w, h, 0, 0) = std::min((double)(occupancy[w * img_h + h]), 1.0);
        }
    }

    //CreateCImgDebugWindow(rgb_stream, "First Image");

    image_pyramid.push_back(std::make_pair(rgb_stream, occupancy_stream));

    //Construct an image pyramid
    while ((img_w > 1) || (img_h > 1))
    {
        img_w = (img_w + 1) / 2;
        img_h = (img_h + 1) / 2;

        rgb_stream.mul(occupancy_stream);

        unclamped_occupancy = occupancy_stream.get_convolve(pull_kernel, 0, false, 1,
            (int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
            0, 0, 0,
            (int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
            2, 2, 1).crop(0, 0, img_w - 1, img_h - 1, 0);

        //unclamped_occupancy = occupancy_stream.get_resize(img_w, img_h, 1, 1, 3);

        occupancy_stream = unclamped_occupancy.get_min(1.0);

        rgb_stream.convolve(pull_kernel, 0, false, 1, 
            (int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
            0, 0, 0,
            (int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
            2, 2, 1).crop(0, 0, img_w - 1, img_h - 1, 0);

        //rgb_stream.resize(img_w, img_h, 1, 3, 3);

        //CreateCImgDebugWindow(rgb_stream, "Convolve size " + std::to_string(img_w) + ", " + std::to_string(img_h));

#pragma omp parallel for
        for (int w = 0; w < img_w; ++w)
        {
            for (int h = 0; h < img_h; ++h)
            {
                if (unclamped_occupancy(w, h, 0, 0) > 0)
                {
                    double mult_val = occupancy_stream(w, h, 0, 0) / unclamped_occupancy(w, h, 0, 0);
                    rgb_stream(w, h, 0, 0) *= mult_val;
                    rgb_stream(w, h, 0, 1) *= mult_val;
                    rgb_stream(w, h, 0, 2) *= mult_val;
                }
            }
        }

        //rgb_stream.mul(occupancy_stream);
        //rgb_stream.div(unclamped_occupancy);

        //std::cout << "PULL W/H: " << img_w << ", " << img_h << std::endl;

        //CreateCImgDebugWindow(rgb_stream, "Pull size " + std::to_string(img_w) + ", " + std::to_string(img_h));

        image_pyramid.push_back(std::make_pair(rgb_stream, occupancy_stream));
    }

    //Iterate backwards through the pyramid to fill empty space
    for (int i = image_pyramid.size() - 2; i >= 0; --i)
    {
        rgb_stream = image_pyramid[i + 1].first.get_resize(image_pyramid[i].second.width(), image_pyramid[i].second.height(), 1, 3, 3);
        
        rgb_stream.convolve(push_kernel, 0, false, 1,
            (int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
            0, 0, 0,
            (int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
            1, 1, 1,
            1, 1, 1);

        //image_pyramid[i + 1].first.convolve(push_kernel, 0, false, 1,
        //    (int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
        //    0, 0, 0,
        //    (int)(~0U >> 1), (int)(~0U >> 1), (int)(~0U >> 1),
        //    1, 1, 1,
        //    1, 1, 1);
        //
        //rgb_stream = image_pyramid[i + 1].first.get_resize(image_pyramid[i].second.width(), image_pyramid[i].second.height(), 1, 3, 1);

        img_w = rgb_stream.width();
        img_h = rgb_stream.height();

        //std::cout << "PUSH W/H: " << img_w << ", " << img_h << 
        //    " (should be " << image_pyramid[i].second.width() << ", " << image_pyramid[i].second.height() << ")" << std::endl;

        rgb_stream.crop(0, 0, image_pyramid[i].second.width() - 1, image_pyramid[i].second.height() - 1, 0);

        rgb_stream.mul(1 - image_pyramid[i].second);
        image_pyramid[i].first.mul(image_pyramid[i].second);

        image_pyramid[i].first += rgb_stream;

        //CreateCImgDebugWindow(image_pyramid[i].first, "Push size " + std::to_string(img_w) + ", " + std::to_string(img_h));
    }

    img_w = tex.width();
    img_h = tex.height();

    tex = image_pyramid[0].first * 255.0;

    //for (size_t w = 0; w < img_w; ++w)
    //{
    //    for (size_t h = 0; h < img_h; ++h)
    //    {
    //        tex(w, h, 0, 0) = image_pyramid[0].first(w, h, 0, 0) * 255.0;
    //        tex(w, h, 0, 1) = image_pyramid[0].first(w, h, 0, 1) * 255.0;
    //        tex(w, h, 0, 2) = image_pyramid[0].first(w, h, 0, 2) * 255.0;
    //    }
    //}
}

bool TextureRemapper::Remap(
    VV_Mesh& original_mesh, VV_Mesh& reparameterized_mesh, 
    cimg_library::CImg<unsigned char>& original_texture, cimg_library::CImg<unsigned char>& remapped_texture, double uv_epsilon, 
    int pad_kernel_size, double pad_kernel_scale, int extra_pad_loops)
{
    std::vector<bool> filled_map(remapped_texture.width() * remapped_texture.height(), false);

    Eigen::Vector2d inverse_scaling(1.0 / remapped_texture.width(), 1.0 / remapped_texture.height());

    double uv_epsilon_sqr = uv_epsilon * uv_epsilon;

    auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(original_mesh.vertices);
    auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, original_mesh.vertices);
    vcm.CleanMeshCGAL(*cgal_mesh);
    auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

    auto cgal_mesh_uv = vcm_uv.GenerateCGAL_MeshFromAttribute(reparameterized_mesh.uvs);
    auto index_remap_uv = vcm_uv.CGAL_To_VV_IndexMap(*cgal_mesh_uv, reparameterized_mesh.uvs);
    vcm_uv.CleanMeshCGAL(*cgal_mesh_uv);
    auto aabb_tree_uv = vcm_uv.CreateAABB(*cgal_mesh_uv);

#pragma omp parallel for
    for (int w = 0; w < remapped_texture.width(); ++w)
    {
        for (int h = 0; h < remapped_texture.height(); ++h)
        {
            Eigen::Vector2d remap_uv_pos((w + 0.5) * inverse_scaling.x(), (h + 0.5) * inverse_scaling.y());

            Eigen::Vector3d barycentric_coords_remap;
            Eigen::Vector3d barycentric_coords_original;
            size_t hit_triangle;
            size_t triangle_index_original;

            vcm_uv.FindClosestPointCGAL(*cgal_mesh_uv, *aabb_tree_uv, remap_uv_pos, hit_triangle, barycentric_coords_remap);

            Eigen::Vector3i* tri_uvs = &(reparameterized_mesh.uvs.indices[(*index_remap_uv)[hit_triangle]]);
            Eigen::Vector2d recreated_uv = reparameterized_mesh.uvs.elements[tri_uvs->x()] * barycentric_coords_remap.x()
                + reparameterized_mesh.uvs.elements[tri_uvs->y()] * barycentric_coords_remap.y()
                + reparameterized_mesh.uvs.elements[tri_uvs->z()] * barycentric_coords_remap.z();

            if ((recreated_uv - remap_uv_pos).squaredNorm() > uv_epsilon_sqr)
            {
                continue;
            }

            Eigen::Vector3i* tri_verts = &(reparameterized_mesh.vertices.indices[(*index_remap_uv)[hit_triangle]]);
            Eigen::Vector3d interpolated_position = reparameterized_mesh.vertices.elements[tri_verts->x()] * barycentric_coords_remap.x()
                + reparameterized_mesh.vertices.elements[tri_verts->y()] * barycentric_coords_remap.y()
                + reparameterized_mesh.vertices.elements[tri_verts->z()] * barycentric_coords_remap.z();

            vcm.FindClosestPointCGAL(*cgal_mesh, *aabb_tree, interpolated_position, triangle_index_original, barycentric_coords_original);

            Eigen::Vector3i* closest_triangle_original = &(original_mesh.uvs.indices[(*index_remap)[triangle_index_original]]);
            Eigen::Vector2d original_uv_pos = original_mesh.uvs.elements[closest_triangle_original->x()] * barycentric_coords_original.x()
                + original_mesh.uvs.elements[closest_triangle_original->y()] * barycentric_coords_original.y()
                + original_mesh.uvs.elements[closest_triangle_original->z()] * barycentric_coords_original.z();

            CopyPoint(original_texture, remapped_texture, original_uv_pos, w, h);

            size_t pixel_array_loc = (size_t)h + (size_t)remapped_texture.height() * (size_t)w;
            filled_map[pixel_array_loc] = true;
        }
    }
    
    PadTexture(remapped_texture, filled_map, extra_pad_loops);

    PadTextureWithPushPull(remapped_texture, filled_map, 
        GetGaussianKernel(pad_kernel_size, pad_kernel_scale, false), 
        GetGaussianKernel(pad_kernel_size, pad_kernel_scale, true));

    remapped_texture.mirror('y');

    return true;
}

bool TextureRemapper::FastRemap(VV_Mesh& original_mesh, VV_Mesh& reparameterized_mesh, 
    cimg_library::CImg<unsigned char>& original_texture, cimg_library::CImg<unsigned char>& remapped_texture, double uv_epsilon, 
    int pad_kernel_size, double pad_kernel_scale, int extra_pad_loops)
{
    std::vector<bool> filled_map(remapped_texture.width() * remapped_texture.height(), false);

    Eigen::Vector2d inverse_scaling(1.0 / remapped_texture.width(), 1.0 / remapped_texture.height());
    double uv_epsilon_sqr = uv_epsilon * uv_epsilon;

    auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(original_mesh.vertices);
    auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, original_mesh.vertices);
    vcm.CleanMeshCGAL(*cgal_mesh);
    auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

    Eigen::Vector2i tex_bound_max = Eigen::Vector2i(remapped_texture.width(), remapped_texture.height());

    std::chrono::high_resolution_clock::time_point tp_now;

    tp_now = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < reparameterized_mesh.uvs.indices.size(); ++i)
    {
        Eigen::Vector3i* tri = &(reparameterized_mesh.uvs.indices[i]);

        Eigen::Vector2d* p0 = &(reparameterized_mesh.uvs.elements[tri->x()]);
        Eigen::Vector2d* p1 = &(reparameterized_mesh.uvs.elements[tri->y()]);
        Eigen::Vector2d* p2 = &(reparameterized_mesh.uvs.elements[tri->z()]);

        Eigen::Vector2d max = p0->cwiseMax(p1->cwiseMax(*p2));
        Eigen::Vector2d min = p0->cwiseMin(p1->cwiseMin(*p2));

        Eigen::Vector2i t_max = Eigen::Vector2i(
            ceil(max.x() * remapped_texture.width() + 1), 
            ceil(max.y() * remapped_texture.height() + 1)).cwiseMin(tex_bound_max);
        Eigen::Vector2i t_min = Eigen::Vector2i(
            floor(min.x() * remapped_texture.width() - 1), 
            floor(min.y() * remapped_texture.height() - 1)).cwiseMax(Eigen::Vector2i::Zero());

        for (size_t w = t_min.x(); w < t_max.x(); ++w)
        {
            for (size_t h = t_min.y(); h < t_max.y(); ++h)
            {
                Eigen::Vector2d remap_uv_pos = Eigen::Vector2d(((double)w + 0.5) * inverse_scaling.x(), ((double)h + 0.5) * inverse_scaling.y());

                Eigen::Vector3d barycentric_coords;

                GetBarycentricCoordinatesOfTriangle(remap_uv_pos, *p0, *p1, *p2, barycentric_coords);

                Eigen::Vector3d clamped_barycentric_coords = barycentric_coords.cwiseMax(Eigen::Vector3d::Zero()).cwiseMin(Eigen::Vector3d::Ones());

                Eigen::Vector3i* tri_uvs = &(reparameterized_mesh.uvs.indices[i]);
                Eigen::Vector2d recreated_uv = reparameterized_mesh.uvs.elements[tri_uvs->x()] * clamped_barycentric_coords.x()
                    + reparameterized_mesh.uvs.elements[tri_uvs->y()] * clamped_barycentric_coords.y()
                    + reparameterized_mesh.uvs.elements[tri_uvs->z()] * clamped_barycentric_coords.z();

                if ((recreated_uv - remap_uv_pos).squaredNorm() > uv_epsilon_sqr)
                {
                    continue;
                }

                Eigen::Vector3i* tri_verts = &(reparameterized_mesh.vertices.indices[i]);
                Eigen::Vector3d interpolated_position = reparameterized_mesh.vertices.elements[tri_verts->x()] * barycentric_coords.x()
                    + reparameterized_mesh.vertices.elements[tri_verts->y()] * barycentric_coords.y()
                    + reparameterized_mesh.vertices.elements[tri_verts->z()] * barycentric_coords.z();

                size_t triangle_index_original;
                Eigen::Vector3d barycentric_coords_original;

                vcm.FindClosestPointCGAL(*cgal_mesh, *aabb_tree, interpolated_position, triangle_index_original, barycentric_coords_original);

                Eigen::Vector3i* closest_triangle_original = &(original_mesh.uvs.indices[(*index_remap)[triangle_index_original]]);
                Eigen::Vector2d original_uv_pos = original_mesh.uvs.elements[closest_triangle_original->x()] * barycentric_coords_original.x()
                    + original_mesh.uvs.elements[closest_triangle_original->y()] * barycentric_coords_original.y()
                    + original_mesh.uvs.elements[closest_triangle_original->z()] * barycentric_coords_original.z();

                CopyPoint(original_texture, remapped_texture, original_uv_pos, w, h);

                size_t pixel_array_loc = h + (size_t)remapped_texture.height() * w;
                filled_map[pixel_array_loc] = true;
            }
        }
    }

    auto time_to_copy = (std::chrono::high_resolution_clock::now() - tp_now).count() * 0.000000001;

    std::cout << "Time to copy points: " << time_to_copy << std::endl;


    tp_now = std::chrono::high_resolution_clock::now();

    PadTexture(remapped_texture, filled_map, extra_pad_loops);

    PadTextureWithPushPull(remapped_texture, filled_map,
        GetGaussianKernel(pad_kernel_size, pad_kernel_scale, false),
        GetGaussianKernel(pad_kernel_size, pad_kernel_scale, true));

    auto time_to_pad = (std::chrono::high_resolution_clock::now() - tp_now).count() * 0.000000001;

    std::cout << "Time to pad points: " << time_to_pad << std::endl;

    remapped_texture.mirror('y');

    return true;
}

bool TextureRemapper::RemapWithPartitions(VV_Mesh& original_mesh, VV_Mesh& reparameterized_mesh, 
    cimg_library::CImg<unsigned char>& original_texture, cimg_library::CImg<unsigned char>& remapped_texture, 
    Eigen::Vector2i partition_count, double partition_buffer, int padding_loops)
{
    if (!LazyUV_Generation(reparameterized_mesh, partition_count, partition_buffer))
    {
        return false;
    }

    std::vector<bool> filled_map(remapped_texture.width() * remapped_texture.height(), false);

    Eigen::Vector2d remap_uv_pos;
    Eigen::Vector2d original_uv_pos;
    Eigen::Vector2d inverse_scaling(1.0 / remapped_texture.width(), 1.0 / remapped_texture.height());
    Eigen::Vector2d scaling(remapped_texture.width(), remapped_texture.height());
    Eigen::Vector2d scaling_minus_one(remapped_texture.width() - 1, remapped_texture.height() - 1);

    Eigen::Vector3i* tri_uvs = nullptr;
    Eigen::Vector3i* tri_verts = nullptr;

    Eigen::Vector2d* p0 = nullptr;
    Eigen::Vector2d* p1 = nullptr;
    Eigen::Vector2d* p2 = nullptr;

    Eigen::Vector2d recreated_uv;
    Eigen::Vector3d barycentric_coords_remap;
    Eigen::Vector3d interpolated_position;

    size_t triangle_index_original;
    size_t hit_triangle;
    Eigen::Vector3i* closest_triangle_original = nullptr;
    Eigen::Vector3d barycentric_coords_original;

    size_t pixel_array_loc = 0;
    size_t filled_count = 0;

    Eigen::Vector2d lower_bound;
    Eigen::Vector2i lower_bound_int;
    Eigen::Vector2d upper_bound;
    Eigen::Vector2i upper_bound_int;

    //double uv_epsilon_sqr = uv_epsilon * uv_epsilon;

    auto cgal_mesh = vcm.GenerateCGAL_MeshFromAttribute(original_mesh.vertices);
    auto index_remap = vcm.CGAL_To_VV_IndexMap(*cgal_mesh, original_mesh.vertices);
    vcm.CleanMeshCGAL(*cgal_mesh);
    auto aabb_tree = vcm.CreateAABB(*cgal_mesh);

    for (int i = 0; i < reparameterized_mesh.uvs.indices.size(); ++i)
    {
        tri_uvs = &(reparameterized_mesh.uvs.indices[i]);

        p0 = &reparameterized_mesh.uvs.elements[tri_uvs->x()];
        p1 = &reparameterized_mesh.uvs.elements[tri_uvs->y()];
        p2 = &reparameterized_mesh.uvs.elements[tri_uvs->z()];

        lower_bound = (p0->cwiseMin(p1->cwiseMin(*p2)).cwiseProduct(scaling)).cwiseMax(0);
        upper_bound = (p0->cwiseMax(p1->cwiseMax(*p2)).cwiseProduct(scaling)).cwiseMin(scaling);

        lower_bound_int.x() = std::floor(lower_bound.x() + 0.5);
        upper_bound_int.x() = std::floor(upper_bound.x() + 0.5);
        lower_bound_int.y() = std::floor(lower_bound.y() + 0.5);
        upper_bound_int.y() = std::floor(upper_bound.y() + 0.5);

        for (int w = lower_bound_int.x(); w < upper_bound_int.x(); ++w)
        {
            remap_uv_pos.x() = ((double)w + 0.5) * inverse_scaling.x();

            for (int h = lower_bound_int.y(); h < upper_bound_int.y(); ++h)
            {
                remap_uv_pos.y() = ((double)h + 0.5) * inverse_scaling.y();

                GetBarycentricCoordinatesOfTriangle(remap_uv_pos, *p0, *p1, *p2, barycentric_coords_remap);

                if (!PointInsideTriangle(barycentric_coords_remap))
                {
                    continue;
                }

                tri_uvs = &(reparameterized_mesh.vertices.indices[i]);
                interpolated_position = reparameterized_mesh.vertices.elements[tri_uvs->x()] * barycentric_coords_remap.x() 
                    + reparameterized_mesh.vertices.elements[tri_uvs->y()] * barycentric_coords_remap.y() 
                    + reparameterized_mesh.vertices.elements[tri_uvs->z()] * barycentric_coords_remap.z();

                vcm.FindClosestPointCGAL(*cgal_mesh, *aabb_tree, interpolated_position, triangle_index_original, barycentric_coords_original);

                closest_triangle_original = &(original_mesh.uvs.indices[(*index_remap)[triangle_index_original]]);
                original_uv_pos = original_mesh.uvs.elements[closest_triangle_original->x()] * barycentric_coords_original.x()
                    + original_mesh.uvs.elements[closest_triangle_original->y()] * barycentric_coords_original.y()
                    + original_mesh.uvs.elements[closest_triangle_original->z()] * barycentric_coords_original.z();

                CopyPoint(original_texture, remapped_texture, original_uv_pos, w, remapped_texture.height() - h - 1);

                pixel_array_loc = (remapped_texture.height() - h - 1) + remapped_texture.height() * w;
                filled_map[pixel_array_loc] = true;
            }
        }

    }

    PadTexture(remapped_texture, filled_map, padding_loops);

    return true;
}
