#pragma once

#include"VV_Mesh.h"
#include <string>

#include "VV_CGAL_Marshaller.h"

#include <CImg.h>

class TextureRemapper
{

	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_3, Eigen::Vector3d> vcm;
	VV_CGAL_Marshaller<CGAL::Simple_cartesian<double>::Point_2, Eigen::Vector2d> vcm_uv;

	void CopyPoint(
		cimg_library::CImg<unsigned char>& original_texture, cimg_library::CImg<unsigned char>& remapped_texture, 
		Eigen::Vector2d &original_uv, int remap_w, int remap_h);

	Eigen::Vector3i GetPixel(cimg_library::CImg<unsigned char>& tex, int x, int y);
	Eigen::Vector3d GetPixelDouble(cimg_library::CImg<unsigned char>& tex, int x, int y);
	void SetPixel(cimg_library::CImg<unsigned char>& tex, int x, int y, Eigen::Vector3i value);
	void SetPixelDouble(cimg_library::CImg<unsigned char>& tex, int x, int y, Eigen::Vector3d value);

	bool LazyUV_Generation(VV_Mesh &to_apply_uvs, Eigen::Vector2i uv_divs, double uv_buffer);

	template<typename T>
	void CreateCImgDebugWindow(cimg_library::CImg<T> &to_display, std::string debug_msg);

public:
	void PadTexture(cimg_library::CImg<unsigned char>& tex, std::vector<bool>& occupancy, int loops);

	cimg_library::CImg<double> GetGaussianKernel(int width, double scale, bool normalized);

	void PadTextureWithPushPull(cimg_library::CImg<unsigned char>& tex, std::vector<bool>& occupancy, 
		cimg_library::CImg<double> pull_kernel, cimg_library::CImg<double> push_kernel);

	bool Remap(
		VV_Mesh &original_mesh, VV_Mesh &reparameterized_mesh, 
		cimg_library::CImg<unsigned char> &original_texture, cimg_library::CImg<unsigned char>& remapped_texture, double uv_epsilon,
		int pad_kernel_size = 5, double pad_kernel_scale = 2.0, int extra_pad_loops = 0);

	bool RemapWithPartitions(
		VV_Mesh& original_mesh, VV_Mesh& reparameterized_mesh,
		cimg_library::CImg<unsigned char>& original_texture, cimg_library::CImg<unsigned char>& remapped_texture, 
		Eigen::Vector2i partition_count, double partition_buffer, int padding_loops);
};

template<typename T>
inline void TextureRemapper::CreateCImgDebugWindow(cimg_library::CImg<T>& to_display, std::string debug_msg)
{
	cimg_library::CImgDisplay debug_disp(to_display, debug_msg.c_str());

	while (!debug_disp.is_closed())
	{
		debug_disp.wait();
	}
}
