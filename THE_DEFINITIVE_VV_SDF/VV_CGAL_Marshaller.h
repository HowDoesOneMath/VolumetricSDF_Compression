#pragma once

#include "VV_Mesh.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Face_count_ratio_stop_predicate.h>

#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>

//#include <CGAL/Polygon_mesh_processing/triangulate_hole.h> 
//#include <CGAL/Polygon_mesh_processing/border.h>
//#include <CGAL/Polygon_mesh_processing/repair.h> // <- Evil
//#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
//#include <CGAL/boost/graph/iterator.h>
//#include <boost/lexical_cast.hpp>

#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>

struct VV_CGAL_Marshaller_Point3VPM {
	using key_type = CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>::Vertex_index;
	using value_type = CGAL::Simple_cartesian<double>::Point_3;
	using reference = value_type;
	using category = boost::readable_property_map_tag;

	const CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>* mesh_ptr;

	VV_CGAL_Marshaller_Point3VPM()
		: mesh_ptr(nullptr)
	{}

	VV_CGAL_Marshaller_Point3VPM(const CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>& m)
		: mesh_ptr(&m)
	{}

	friend VV_CGAL_Marshaller_Point3VPM::value_type get(const VV_CGAL_Marshaller_Point3VPM& map, CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>::Vertex_index idx)
	{
		CGAL::Simple_cartesian<double>::Point_2 p = map.mesh_ptr->point(idx);
		return { p[0], p[1], 0 };
	}

};

template<typename DIM_CGAL, typename DIM_VV>
class VV_CGAL_Marshaller
{
public:
	std::shared_ptr<CGAL::Surface_mesh<DIM_CGAL>> GenerateCGAL_MeshFromAttribute(VV_Mesh_Attribute<DIM_VV>& att);

	std::shared_ptr<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>, 
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>> 
		CreateAABB(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> &sm);

	std::shared_ptr<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>, VV_CGAL_Marshaller_Point3VPM>>>>
		CreateAABB(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>& sm);

	std::shared_ptr<std::vector<size_t>> CGAL_To_VV_IndexMap(CGAL::Surface_mesh<DIM_CGAL>& sm, AbstractAttribute& att);

	bool CopyCGAL_To_VV_Attribute(CGAL::Surface_mesh<DIM_CGAL>& sm, VV_Mesh_Attribute<DIM_VV>& att);

	std::shared_ptr<CGAL::Surface_mesh<DIM_CGAL>> DecimateCGAL_Mesh(CGAL::Surface_mesh<DIM_CGAL>& sm, double edge_ratio);

	void RaycastCGAL(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& sm, CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>&aabb,
		Eigen::Vector3d point, Eigen::Vector3d direction, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords);

	std::shared_ptr<std::vector<std::pair<size_t, Eigen::Vector3d>>> RaycastCGAL(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& sm,
		CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>&aabb,
		Eigen::Vector3d point, Eigen::Vector3d direction);

	void FindClosestPointCGAL(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& sm, CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>& aabb, 
		Eigen::Vector3d point, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords);

	void FindClosestPointCGAL(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>& sm, CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>, VV_CGAL_Marshaller_Point3VPM>>>&aabb,
		Eigen::Vector2d point, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords);

	std::shared_ptr<std::vector<Eigen::Vector3d>> GetMeshDisplacements(VV_Mesh& mod_mesh, VV_Mesh& src_mesh,
		CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& src_sm, std::vector<size_t>& src_cgal_remap,
		CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>&src_aabb);

	void CleanMeshCGAL(CGAL::Surface_mesh<DIM_CGAL>& sm);

	void RepairMesh(CGAL::Surface_mesh<DIM_CGAL> &mesh);
};

template<typename DIM_CGAL, typename DIM_VV>
inline std::shared_ptr<CGAL::Surface_mesh<DIM_CGAL>> VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::GenerateCGAL_MeshFromAttribute(VV_Mesh_Attribute<DIM_VV>& att)
{
	std::shared_ptr<CGAL::Surface_mesh<DIM_CGAL>> sm = std::make_shared<CGAL::Surface_mesh<DIM_CGAL>>();

	std::vector<DIM_CGAL> elems;
	elems.resize(att.elements.size());

	auto elems_mem_size = elems.size() * sizeof(DIM_CGAL);

	memcpy(elems.data(), att.elements.data(), elems_mem_size);

	CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(elems, att.indices, *sm);

	return sm;
}

template<typename DIM_CGAL, typename DIM_VV>
inline std::shared_ptr<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>, 
	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>> 
	VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::CreateAABB(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& sm)
{
	std::shared_ptr<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>> to_return =
		std::make_shared<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>>();

	CGAL::Polygon_mesh_processing::build_AABB_tree(sm, *to_return);

	return to_return;
}

template<typename DIM_CGAL, typename DIM_VV>
inline std::shared_ptr<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>, VV_CGAL_Marshaller_Point3VPM>>>>
	VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::CreateAABB(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>& sm)
{
	std::shared_ptr<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>, VV_CGAL_Marshaller_Point3VPM>>>> to_return =
		std::make_shared<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
		CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>, VV_CGAL_Marshaller_Point3VPM>>>>();

	VV_CGAL_Marshaller_Point3VPM vpm(sm);

	CGAL::Polygon_mesh_processing::build_AABB_tree(sm, *to_return, CGAL::parameters::vertex_point_map(vpm));

	return to_return;
}

template<typename DIM_CGAL, typename DIM_VV>
inline std::shared_ptr<std::vector<size_t>> VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::CGAL_To_VV_IndexMap(CGAL::Surface_mesh<DIM_CGAL>& sm, AbstractAttribute& att)
{
	std::shared_ptr<std::vector<size_t>> to_return = std::make_shared<std::vector<size_t>>();

	if (sm.number_of_faces() <= 0)
	{
		std::cout << "ERROR: NO SURFACE MESH FACES! " << std::endl;
		return to_return;
	}

	to_return->resize(sm.number_of_faces(), SIZE_MAX);

	size_t cgal_remap_index = 0;
	Eigen::Vector3i cgal_face;

	auto sm_inds = sm.vertices_around_face(sm.halfedge(CGAL::Surface_mesh<DIM_CGAL>::Face_index(cgal_remap_index)));
	int t = 0;

	for (auto cgal_index : sm_inds)
	{
		cgal_face[t] = cgal_index.id();
		++t;
	}

	for (size_t vv_remap_index = 0; vv_remap_index < att.indices.size(); ++vv_remap_index)
	{
		if (att.indices[vv_remap_index] == cgal_face)
		{
			(*to_return)[cgal_remap_index] = vv_remap_index;

			++cgal_remap_index;

			if (cgal_remap_index >= sm.number_of_faces())
			{
				break;
			}

			sm_inds = sm.vertices_around_face(sm.halfedge(CGAL::Surface_mesh<DIM_CGAL>::Face_index(cgal_remap_index)));

			t = 0;
			for (auto cgal_index : sm_inds)
			{
				cgal_face[t] = cgal_index.id();
				++t;
			}
		}
	}

	//std::cout << "Faces CGAL: " << sm.number_of_faces() << ", removed CGAL: " << sm.number_of_removed_faces() << std::endl;

	if (cgal_remap_index < sm.number_of_faces())
	{

		std::cout << "ERROR: COULD NOT FIND CGAL FACES THAT MATCH VV FACES! " << std::endl;
	}

	return to_return;
}

template<typename DIM_CGAL, typename DIM_VV>
inline bool VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::CopyCGAL_To_VV_Attribute(CGAL::Surface_mesh<DIM_CGAL>& sm, VV_Mesh_Attribute<DIM_VV>& att)
{
	att.elements.resize(sm.number_of_vertices());
	att.indices.resize(sm.number_of_faces());

	for (size_t i = 0; i < sm.number_of_vertices(); ++i)
	{
		auto vert = sm.point(CGAL::Surface_mesh<DIM_CGAL>::Vertex_index(i));
		memcpy(&(att.elements[i]), &vert, sizeof(DIM_CGAL));
	}

	for (size_t i = 0; i < sm.number_of_faces(); ++i)
	{
		auto sm_inds = sm.vertices_around_face(sm.halfedge(CGAL::Surface_mesh<DIM_CGAL>::Face_index(i)));
		int t = 0;

		for (auto cgal_index : sm_inds)
		{
			att.indices[i][t] = cgal_index.id();
			++t;
		}
	}

	return true;
}

template<typename DIM_CGAL, typename DIM_VV>
inline std::shared_ptr<CGAL::Surface_mesh<DIM_CGAL>> VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::DecimateCGAL_Mesh(CGAL::Surface_mesh<DIM_CGAL>& sm, double edge_ratio)
{
	std::shared_ptr<CGAL::Surface_mesh<DIM_CGAL>> to_return = std::make_shared<CGAL::Surface_mesh<DIM_CGAL>>();

	*to_return = sm;

	CGAL::Surface_mesh_simplification::Edge_count_ratio_stop_predicate<CGAL::Surface_mesh<DIM_CGAL>> stop(edge_ratio);
	//CGAL::Surface_mesh_simplification::Face_count_ratio_stop_predicate<CGAL::Surface_mesh<DIM_CGAL>> stop(edge_ratio, *to_return);
	
	//int r = CGAL::Surface_mesh_simplification::edge_collapse(*to_return, stop);

	//How to use GH surface simplification:
	// Pick 1 of 4:
	//CGAL::Surface_mesh_simplification::GarlandHeckbert_plane_policies<CGAL::Surface_mesh<DIM_CGAL>, 
	//	CGAL::Simple_cartesian<double>> gh_policies(*to_return);
	CGAL::Surface_mesh_simplification::GarlandHeckbert_triangle_policies<CGAL::Surface_mesh<DIM_CGAL>, 
		CGAL::Simple_cartesian<double>> gh_policies(*to_return);
	//CGAL::Surface_mesh_simplification::GarlandHeckbert_probabilistic_plane_policies<CGAL::Surface_mesh<DIM_CGAL>, 
	//	CGAL::Simple_cartesian<double>> gh_policies(*to_return);
	//CGAL::Surface_mesh_simplification::GarlandHeckbert_probabilistic_triangle_policies<CGAL::Surface_mesh<DIM_CGAL>, 
	//	CGAL::Simple_cartesian<double>> gh_policies(*to_return);
	// 
	// Uncomment:
	const auto& gh_cost = gh_policies.get_cost();
	const auto& gh_placement = gh_policies.get_placement();
	// 
	// Pick 1 of 4:
	//CGAL::Surface_mesh_simplification::Bounded_normal_change_placement<
	//	CGAL::Surface_mesh_simplification::GarlandHeckbert_plane_policies<CGAL::Surface_mesh<DIM_CGAL>,
	//	CGAL::Simple_cartesian<double>>::Get_placement> placement(gh_placement);
	CGAL::Surface_mesh_simplification::Bounded_normal_change_placement<
		CGAL::Surface_mesh_simplification::GarlandHeckbert_triangle_policies<CGAL::Surface_mesh<DIM_CGAL>,
		CGAL::Simple_cartesian<double>>::Get_placement> placement(gh_placement);
	//CGAL::Surface_mesh_simplification::Bounded_normal_change_placement<
	//	CGAL::Surface_mesh_simplification::GarlandHeckbert_probabilistic_plane_policies<CGAL::Surface_mesh<DIM_CGAL>,
	//	CGAL::Simple_cartesian<double>>::Get_placement> placement(gh_placement);
	//CGAL::Surface_mesh_simplification::Bounded_normal_change_placement<
	//	CGAL::Surface_mesh_simplification::GarlandHeckbert_probabilistic_triangle_policies<CGAL::Surface_mesh<DIM_CGAL>,
	//	CGAL::Simple_cartesian<double>>::Get_placement> placement(gh_placement);
	//
	// Uncomment:
	int r = CGAL::Surface_mesh_simplification::edge_collapse(*to_return, stop,
		CGAL::parameters::get_cost(gh_cost)
		.get_placement(placement));

	to_return->collect_garbage();

	return to_return;
}

template<typename DIM_CGAL, typename DIM_VV>
inline void VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::RaycastCGAL(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& sm, 
	CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>, 
	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>& aabb, 
	Eigen::Vector3d point, Eigen::Vector3d direction, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords)
{
	CGAL::Simple_cartesian<double>::Ray_3 ray(
		*reinterpret_cast<CGAL::Simple_cartesian<double>::Point_3*>(&point),
		*reinterpret_cast<CGAL::Simple_cartesian<double>::Direction_3*>(&direction));
	auto closest = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(ray, aabb, sm);

	out_triangle_index = closest.first;

	barycentric_coords.x() = std::clamp(closest.second[1], 0.0, 1.0);
	barycentric_coords.y() = std::clamp(closest.second[2], 0.0, 1.0);
	barycentric_coords.z() = std::clamp(closest.second[0], 0.0, 1.0);
}

//DO NOT USE THIS OVERLOAD, IT DOES NOTHING! USE THE OTHER ONE!
template<typename DIM_CGAL, typename DIM_VV>
inline std::shared_ptr<std::vector<std::pair<size_t, Eigen::Vector3d>>> VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::RaycastCGAL(
	CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& sm, CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>, 
	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>& aabb, Eigen::Vector3d point, Eigen::Vector3d direction)
{
	auto to_return = std::make_shared<std::vector<std::pair<size_t, Eigen::Vector3d>>>();

	//CGAL::Simple_cartesian<double>::Ray_3 ray(
	//	*reinterpret_cast<CGAL::Simple_cartesian<double>::Point_3*>(&point),
	//	*reinterpret_cast<CGAL::Simple_cartesian<double>::Direction_3*>(&direction));
	//
	//std::vector<CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
	//	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>
	//	::Intersection_and_primitive_id<CGAL::Simple_cartesian<double>::Ray_3>::Type> intersections;
	//
	//aabb.all_intersections(ray, std::back_inserter(intersections));
	//
	//to_return->resize(intersections.size());
	//
	//CGAL::Simple_cartesian<double>::Point_3* p;
	//
	//for (size_t i = 0; i < to_return->size(); ++i)
	//{
	//	(*to_return)[i].first = intersections[i].second;
	//	
	//	p = boost::get<CGAL::Simple_cartesian<double>::Point_3>(&intersections[i].second);
	//	(*to_return)[i].second.x() = std::clamp(intersections[i].first., 0.0, 1.0);
	//}

	return to_return;
}

template<typename DIM_CGAL, typename DIM_VV>
inline void VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::FindClosestPointCGAL(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& sm, 
	CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>&aabb,
	Eigen::Vector3d point, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords)
{
	auto closest = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(*reinterpret_cast<CGAL::Simple_cartesian<double>::Point_3*>(&point), aabb, sm);

	out_triangle_index = closest.first;

	barycentric_coords.x() = std::clamp(closest.second[1], 0.0, 1.0);
	barycentric_coords.y() = std::clamp(closest.second[2], 0.0, 1.0);
	barycentric_coords.z() = std::clamp(closest.second[0], 0.0, 1.0);
}

template<typename DIM_CGAL, typename DIM_VV>
inline void VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::FindClosestPointCGAL(CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>& sm, 
	CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>,
	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_2>, VV_CGAL_Marshaller_Point3VPM>>>& aabb,
	Eigen::Vector2d point, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords)
{
	auto closest = CGAL::Polygon_mesh_processing::locate_with_AABB_tree(*reinterpret_cast<CGAL::Simple_cartesian<double>::Point_2*>(&point), aabb, sm);

	out_triangle_index = closest.first;

	barycentric_coords.x() = closest.second[1];
	barycentric_coords.y() = closest.second[2];
	barycentric_coords.z() = closest.second[0];
}

template<typename DIM_CGAL, typename DIM_VV>
inline std::shared_ptr<std::vector<Eigen::Vector3d>> VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::GetMeshDisplacements(VV_Mesh& mod_mesh, VV_Mesh& src_mesh,
	CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>& src_sm, std::vector<size_t> &src_cgal_remap, 
	CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>, 
	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>>& src_aabb)
{
	std::shared_ptr<std::vector<Eigen::Vector3d>> to_return = std::make_shared<std::vector<Eigen::Vector3d>>();

	to_return->resize(mod_mesh.vertices.elements.size());

	//std::shared_ptr<VV_Mesh> to_return = std::make_shared<VV_Mesh>();

	//to_return->vertices.CopyFrom(mod_mesh.vertices);
	//to_return->uvs.CopyFrom(mod_mesh.uvs);

	//if (src_mesh.normals.indices.size() > 0)
	//{
	//	mod_mesh.normals.elements.resize(mod_mesh.vertices.elements.size());
	//	mod_mesh.normals.indices = mod_mesh.vertices.indices;
	//}

	size_t out_index;
	Eigen::Vector3d interpolated_position;
	Eigen::Vector3d interpolated_normal;
	Eigen::Vector3d barycentric_coords;
	Eigen::Vector3i* tri_verts = nullptr;
	Eigen::Vector3i* tri_norms = nullptr;

	for (size_t i = 0; i < mod_mesh.vertices.elements.size(); ++i)
	{
		FindClosestPointCGAL(src_sm, src_aabb, mod_mesh.vertices.elements[i], out_index, barycentric_coords);

		tri_verts = &(src_mesh.vertices.indices[src_cgal_remap[out_index]]);
		interpolated_position = src_mesh.vertices.elements[tri_verts->x()] * barycentric_coords.x()
			+ src_mesh.vertices.elements[tri_verts->y()] * barycentric_coords.y()
			+ src_mesh.vertices.elements[tri_verts->z()] * barycentric_coords.z();

		(*to_return)[i] = interpolated_position - mod_mesh.vertices.elements[i];

		//mod_mesh.vertices.elements[i] = interpolated_position;

		//if (src_mesh.normals.indices.size() > 0)
		//{
		//	tri_norms = &(src_mesh.normals.indices[src_cgal_remap[out_index]]);
		//	interpolated_normal = src_mesh.normals.elements[tri_norms->x()] * barycentric_coords.x()
		//		+ src_mesh.normals.elements[tri_norms->y()] * barycentric_coords.y()
		//		+ src_mesh.normals.elements[tri_norms->z()] * barycentric_coords.z();
		//	mod_mesh.normals.elements[i] = interpolated_normal.normalized();
		//}
	}

	return to_return;
}

template<typename DIM_CGAL, typename DIM_VV>
inline void VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::CleanMeshCGAL(CGAL::Surface_mesh<DIM_CGAL>& sm)
{
	DIM_CGAL p[3];

	for (size_t i = 0; i < sm.number_of_faces(); ++i)
	{
		auto face_index = CGAL::Surface_mesh<DIM_CGAL>::Face_index(i);
		auto sm_inds = sm.vertices_around_face(sm.halfedge(face_index));
		int t = 0;

		for (auto vert_index : sm_inds)
		{
			p[t] = sm.point(vert_index);
			++t;
		}

		if (p[0] == p[1] || p[0] == p[2] || p[1] == p[2])
		{
			sm.remove_face(face_index);
		}
	}

	//sm.collect_garbage();
}

template<typename DIM_CGAL, typename DIM_VV>
inline void VV_CGAL_Marshaller<DIM_CGAL, DIM_VV>::RepairMesh(CGAL::Surface_mesh<DIM_CGAL> &mesh)
{
	//I am deeply infuriated at the problems CGAL has delivered to me.

	//unsigned int nb_holes = 0;
	//std::vector<boost::graph_traits<CGAL::Surface_mesh<DIM_CGAL>>::halfedge_descriptor> border_cycles;

	//CGAL::Polygon_mesh_processing::extract_boundary_cycles(mesh, std::back_inserter(border_cycles));
	//for (boost::graph_traits<CGAL::Surface_mesh<DIM_CGAL>>::halfedge_descriptor h : border_cycles)
	//{
	//	std::vector<boost::graph_traits<CGAL::Surface_mesh<DIM_CGAL>>::face_descriptor>  patch_facets;
	//	std::vector<boost::graph_traits<CGAL::Surface_mesh<DIM_CGAL>>::vertex_descriptor> patch_vertices;
	//
	//	CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(mesh,
	//		h,
	//		CGAL::parameters::face_output_iterator(std::back_inserter(patch_facets))
	//		.vertex_output_iterator(std::back_inserter(patch_vertices)));
	//	++nb_holes;
	//}
}
