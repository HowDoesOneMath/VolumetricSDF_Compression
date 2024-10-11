#pragma once

#include "VV_Mesh_Attribute.h"

#include <vector>
#include <string>

#include <unordered_set>

#define _DEBUG

#include <UVAtlas.h>
#include <DirectXMesh.h>
#include <draco/mesh/mesh.h>

#include "BasicGeometry.h"

class VV_Mesh
{
public:

	enum VV_Mesh_Ignore_Flags
	{
		IGNORE_VERTICES = 1,
		IGNORE_UVS = 2,
		IGNORE_NORMALS = 4
	};


	VV_Mesh_Attribute<Eigen::Vector3d> vertices;
	VV_Mesh_Attribute<Eigen::Vector2d> uvs;
	VV_Mesh_Attribute<Eigen::Vector3d> normals;

	//bool cgal_mesh_created = false;
	//CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3> sm;
	//std::vector<int> cgal_to_vv_index;
	//CGAL::AABB_tree<CGAL::AABB_traits<CGAL::Simple_cartesian<double>, 
	//	CGAL::AABB_face_graph_triangle_primitive<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>>>> aabb_tree;

	bool WriteOBJ(std::string filename);

	bool ReadOBJ(std::string filename);

	//std::shared_ptr<CGAL::Surface_mesh<CGAL::Simple_cartesian<double>::Point_3>> GenerateCGAL_MeshFromVerts();
	//void CleanupLocatorCGAL();

	/// <summary>
	/// Clears the mesh and creates a cube with only position attributes.
	/// </summary>
	/// <param name="dims">Dimensions of the cube</param>
	/// <param name="offset">Offset of the cube</param>
	void ApplyDebuggingCube(Eigen::Vector3d dims, Eigen::Vector3d offset);

	//std::shared_ptr<VV_Mesh> DecimateEdges(double decimation_ratio);
	bool GenerateNewUVsWithUVAtlas(size_t width, size_t height, float gutter);

	void SubdivideMesh(int iterations);
	std::shared_ptr<std::vector<std::pair<size_t, std::vector<std::tuple<size_t, size_t, std::unordered_set<size_t>>>>>> SubdivideMeshAndGetAdjacencies(int iterations);
	void SelectivelySubdivideMesh(std::vector<unsigned char> &subdivision_counts);

	void CopyFromMesh(VV_Mesh& other, int ignore_flags = 0);
	std::shared_ptr<VV_Mesh> GetCopy(int ignore_flags = 0);

	std::shared_ptr<VV_Mesh> ExtractSubmesh(std::vector<size_t>& target_indices, int ignore_flags = 0);

	std::shared_ptr<draco::Mesh> ToDracoMesh();
	void FromDracoMesh(draco::Mesh& mesh);

	void ConcatenateMesh(VV_Mesh& other, Eigen::Vector3d offset, int ignore_flags = 0);

	template<typename T>
	void ConcatenateAttribute(VV_Mesh_Attribute<T>& source, VV_Mesh_Attribute<T>& destination, T offset, size_t default_add);

	void ClearUnreferencedElements();

	void NegateTriangle(size_t triangle_index) {
		vertices.indices[triangle_index] = Eigen::Vector3i(-1, -1, -1);

		if (uvs.indices.size() > 0)
			uvs.indices[triangle_index] = Eigen::Vector3i(-1, -1, -1);

		if (normals.indices.size() > 0)
			normals.indices[triangle_index] = Eigen::Vector3i(-1, -1, -1);
	}

	size_t ClearNegativeTriangles() {
		size_t to_return = 0;

		to_return += vertices.ClearNegativeTriangles();
		to_return += uvs.ClearNegativeTriangles();
		to_return += normals.ClearNegativeTriangles();

		return to_return;
	}

	void Clear(int ignore_flags = 0);

	void FindClosestPointGreedy(Eigen::Vector3d point, size_t &out_triangle_index, Eigen::Vector3d &barycentric_coords);
	void FindClosestPointAmongList(std::vector<size_t> &to_check, Eigen::Vector3d point,size_t &out_triangle_index, Eigen::Vector3d &barycentric_coords);
	//void FindClosestPointCGAL(Eigen::Vector3d point, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords);

	std::pair<Eigen::Vector3d, Eigen::Vector3d> GetBoundingBox();

	void JoinOverlappingVertices(double overlapping_threshold);

	void JoinOverlappingVertices();

private:
	template<typename T>
	void AddDracoAttribute(std::shared_ptr<draco::Mesh> mesh, draco::GeometryAttribute::Type type, VV_Mesh_Attribute<T> &attribute);

	template<typename T>
	void RetrieveDracoAttribute(draco::Mesh &mesh, draco::GeometryAttribute::Type type, VV_Mesh_Attribute<T>& attribute);

	void WriteVertices(std::ofstream& savefile);
	void WriteUVs(std::ofstream& savefile);
	void WriteNormals(std::ofstream& savefile);

	void WriteTriangles(std::ofstream& savefile);

public:
	void DebugIrregularities(double very_far_away_threshold);

	void DebugGeneralStats()
	{
		std::cout << "Vertices: \n" << vertices.elements.size() << ", Tris: " << vertices.indices.size() << std::endl;
		std::cout << "UVs: \n" << uvs.elements.size() << ", Tris: " << uvs.indices.size() << std::endl;
		std::cout << "Normals: \n" << normals.elements.size() << ", Tris: " << normals.indices.size() << std::endl;

	}

	size_t FindNonManifoldEdges();

	HRESULT UVAtlasTestCallback(float val)
	{
		std::cout << "\t" << val << std::endl;

		return S_OK;
	}

	//void GetQuadricErrorMetric(int index, int* presizedQuadric);
};

template<typename T>
inline void VV_Mesh::ConcatenateAttribute(VV_Mesh_Attribute<T>& source, VV_Mesh_Attribute<T>& destination, T offset, size_t default_add)
{
	if ((source.indices.size() > 0) && (destination.indices.size() <= 0))
		return;

	if ((source.indices.size() <= 0) && (destination.indices.size() > 0))
	{
		for (size_t i = 0; i < default_add; ++i)
		{
			destination.indices.push_back(Eigen::Vector3i(0, 0, 0));
		}
		return;
	}

	destination.Concatenate(source, offset);
}

template<typename T>
inline void VV_Mesh::AddDracoAttribute(std::shared_ptr<draco::Mesh> mesh, draco::GeometryAttribute::Type type, VV_Mesh_Attribute<T>& attribute)
{
	if (attribute.elements.size() <= 0)
	{
		return;
	}

	size_t dim_count = sizeof(T) / sizeof(double);
	size_t elem_count = attribute.elements.size();

	float* elems = new float[dim_count];

	draco::GeometryAttribute va;
	va.Init(type, nullptr, dim_count, draco::DT_FLOAT32, false, sizeof(float) * dim_count, 0);
	auto att = mesh->AddAttribute(va, false, elem_count);

	for (size_t i = 0; i < elem_count; ++i)
	{
		for (int j = 0; j < dim_count; ++j)
		{
			elems[j] = attribute.elements[i][j];
		}

		mesh->attribute(att)->SetAttributeValue(draco::AttributeValueIndex(i), elems);
	}

	delete[] elems;

	for (size_t i = 0; i < attribute.indices.size(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			const draco::PointIndex vert_id(3 * i + j);
			mesh->attribute(att)->SetPointMapEntry(vert_id, draco::AttributeValueIndex(attribute.indices[i][j]));
		}
	}
}

template<typename T>
inline void VV_Mesh::RetrieveDracoAttribute(draco::Mesh &mesh, draco::GeometryAttribute::Type type, VV_Mesh_Attribute<T>& attribute)
{
	const draco::PointAttribute* const att =
		mesh.GetNamedAttribute(type);
	if (att == nullptr || att->size() == 0) {
		return;
	}

	size_t dim_count = sizeof(T) / sizeof(double);

	attribute.elements.resize(att->size());
	attribute.indices.resize(mesh.num_faces());

	float* vals = new float[dim_count];

	for (draco::AttributeValueIndex i(0); i < static_cast<uint32_t>(att->size()); ++i) {
		att->GetValue(i, vals);

		for (int j = 0; j < dim_count; ++j)
		{
			attribute.elements[i.value()][j] = vals[j];
		}
	}

	for (draco::FaceIndex i(0); i < mesh.num_faces(); ++i) {
		for (int j = 0; j < 3; ++j)
		{
			attribute.indices[i.value()][j] = att->mapped_index(mesh.face(i)[j]).value();
		}
	}

	delete[] vals;
}