#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh_simplification/Edge_collapse_visitor_base.h>

struct DecimationVisitor : CGAL::Surface_mesh_simplification::Edge_collapse_visitor_base<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>
{
	std::vector<size_t> vertex_remap;
	//std::vector<size_t> normal_remap;

	void SetRemapSize(size_t new_size)
	{
		vertex_remap.resize(new_size, -1);
	}

	//TODO: Do manual decimation from CGAL structure!

	//IDEA: Redirect vertices, then copy position values!

	// Called after each edge has been collapsed
	void OnCollapsed(
		const CGAL::Surface_mesh_simplification::Edge_profile<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>& profile,
		boost::graph_traits<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>::vertex_descriptor descriptor)
	{
		//vertex_remap[profile.v0()] = profile.v1();

		//std::cout << "MERGING: " << profile.v0() << ", " << profile.v1() << " --> " << descriptor << std::endl;

		//auto affected = profile.border_edges();
		//
		//for (size_t i = 0; i < affected.size(); ++i)
		//{
		//	auto sm_face = profile.surface_mesh().face(affected[i]);
		//
		//	
		//}
	}
};