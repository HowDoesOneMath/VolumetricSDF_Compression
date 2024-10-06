#include "VV_Mesh.h"

#include <fstream>
#include <iostream>
#include <queue>

#include "AdditionalUtilities.h"

#define VV_MESH_DEBUG_UV_GENERATION 0
//#define VV_MESH_DEBUG_UV_GENERATION 1

bool VV_Mesh::WriteOBJ(std::string filename)
{
	std::ofstream savefile;

	savefile.open(filename);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open writing file for mesh " << std::endl;
		return false;
	}

	WriteTriangles(savefile);

	savefile.close();

	return true;
}

bool VV_Mesh::ReadOBJ(std::string filename)
{
	Clear();

	std::ifstream savefile;

	savefile.open(filename);

	if (!savefile.is_open())
	{
		std::cout << "Couldn't open reading file for mesh " << filename << std::endl;
		return false;
	}

	std::string line;
	std::vector<std::string> attribute_components;

	std::vector<std::string> triangle_components;

	std::vector<std::vector<std::string>> face_components;

	int line_num = 0;

	std::string current_input = "NONE";
	size_t input_count = 0;

	while (std::getline(savefile, line))
	{
		++line_num;
		SplitString(line, attribute_components, " ");

		//std::cout << "Processing line \'" << line << "\'..." << std::endl;

		//if (line_num == 0)
		//{
		//	std::cout << line << std::endl;
		//}

		if (attribute_components[0] == "v")
		{
			vertices.elements.push_back(Eigen::Vector3d(
				std::stod(attribute_components[1]),
				std::stod(attribute_components[2]),
				std::stod(attribute_components[3])));
		}
		else if (attribute_components[0] == "vt")
		{
			uvs.elements.push_back(Eigen::Vector2d(
				std::stod(attribute_components[1]),
				std::stod(attribute_components[2])));
		}
		else if (attribute_components[0] == "vn")
		{
			normals.elements.push_back(Eigen::Vector3d(
				std::stod(attribute_components[1]),
				std::stod(attribute_components[2]),
				std::stod(attribute_components[3])));
		}
		else if (attribute_components[0] == "f")
		{
			for (int i = 1; i < attribute_components.size(); ++i)
			{
				SplitString(attribute_components[i], triangle_components, "\\/", "", false);
				if (triangle_components.size() <= 0)
				{
					std::cout << "ERROR - invalid face component structure on line " << line_num << "!" << std::endl;
					savefile.close();
					return false;
				}

				face_components.push_back(triangle_components);
				triangle_components.clear();
			}

			//std::cout << "Evaluating " << face_components.size() << " components on face " << input_count << ": " << line << std::endl;
			//++input_count;

			for (int i = 0; i < (int)face_components.size() - 2; ++i)
			{
				vertices.indices.push_back(Eigen::Vector3i(
					std::stoi(face_components[0][0]) - 1,
					std::stoi(face_components[i + 1][0]) - 1,
					std::stoi(face_components[i + 2][0]) - 1));

				if (uvs.elements.size() > 0)
				{
					uvs.indices.push_back(Eigen::Vector3i(
						std::stoi(face_components[0][1]) - 1,
						std::stoi(face_components[i + 1][1]) - 1,
						std::stoi(face_components[i + 2][1]) - 1));
				}

				if (normals.elements.size() > 0)
				{
					normals.indices.push_back(Eigen::Vector3i(
						std::stoi(face_components[0][2]) - 1,
						std::stoi(face_components[i + 1][2]) - 1,
						std::stoi(face_components[i + 2][2]) - 1));
				}
			}

			face_components.clear();
		}

		attribute_components.clear();
	}

	//std::cout << "Lines read: " << line_num << std::endl;

	savefile.close();

	//std::cout << "Save file closed" << std::endl;

	return true;
}

void VV_Mesh::ApplyDebuggingCube(Eigen::Vector3d dims, Eigen::Vector3d offset)
{
	Clear();

	vertices.elements.push_back(0.5 * Eigen::Vector3d(dims.x(), dims.y(), dims.z()) + offset);
	vertices.elements.push_back(0.5 * Eigen::Vector3d(dims.x(), dims.y(), -dims.z()) + offset);
	vertices.elements.push_back(0.5 * Eigen::Vector3d(dims.x(), -dims.y(), -dims.z()) + offset);
	vertices.elements.push_back(0.5 * Eigen::Vector3d(dims.x(), -dims.y(), dims.z()) + offset);
	vertices.elements.push_back(0.5 * Eigen::Vector3d(-dims.x(), dims.y(), dims.z()) + offset);
	vertices.elements.push_back(0.5 * Eigen::Vector3d(-dims.x(), dims.y(), -dims.z()) + offset);
	vertices.elements.push_back(0.5 * Eigen::Vector3d(-dims.x(), -dims.y(), -dims.z()) + offset);
	vertices.elements.push_back(0.5 * Eigen::Vector3d(-dims.x(), -dims.y(), dims.z()) + offset);
	
	vertices.indices.push_back(Eigen::Vector3i(0, 3, 2));
	vertices.indices.push_back(Eigen::Vector3i(0, 2, 1));
	vertices.indices.push_back(Eigen::Vector3i(4, 5, 6));
	vertices.indices.push_back(Eigen::Vector3i(4, 6, 7));

	vertices.indices.push_back(Eigen::Vector3i(0, 1, 5));
	vertices.indices.push_back(Eigen::Vector3i(0, 5, 4));
	vertices.indices.push_back(Eigen::Vector3i(3, 7, 6));
	vertices.indices.push_back(Eigen::Vector3i(3, 6, 2));

	vertices.indices.push_back(Eigen::Vector3i(0, 4, 7));
	vertices.indices.push_back(Eigen::Vector3i(0, 7, 3));
	vertices.indices.push_back(Eigen::Vector3i(1, 2, 6));
	vertices.indices.push_back(Eigen::Vector3i(1, 6, 5));
}

bool VV_Mesh::GenerateNewUVsWithUVAtlas(size_t width, size_t height, float gutter)
{
	uvs.Clear();

	HRESULT result;

	size_t nVerts = vertices.elements.size();
	size_t nFaces = vertices.indices.size();

	auto pos = std::make_unique<DirectX::XMFLOAT3[]>(nVerts);
	for (size_t j = 0; j < nVerts; ++j)
	{
		pos[j].x = vertices.elements[j].x();
		pos[j].y = vertices.elements[j].y();
		pos[j].z = vertices.elements[j].z();
	}

	//std::cout << "Generating Adjacency..." << std::endl;

	std::unique_ptr<uint32_t[]> adj(new uint32_t[nFaces * 3]);
	result = DirectX::GenerateAdjacencyAndPointReps(reinterpret_cast<uint32_t*>(vertices.indices.data()), nFaces,
		pos.get(), nVerts, 0.f, nullptr, adj.get());

	if (FAILED(result))
	{
		std::cout << "ADJACENCY FAILED: " << result << " (" << (result - INT_MIN) << ")" << std::endl;
		return false;
	}

	

	std::vector<DirectX::UVAtlasVertex> vb;
	std::vector<uint8_t> ib;
	std::vector<uint32_t> remap;

	//std::cout << "Creating Atlas..." << std::endl;

	//float stretch_amnt = 0.f;
	float stretch_amnt = 0.1f;

	DXGI_FORMAT fmt = DXGI_FORMAT_R32_UINT;
	//DXGI_FORMAT fmt = DXGI_FORMAT_R32_SINT;

	result = DirectX::UVAtlasCreate(pos.get(), nVerts,
		vertices.indices.data(), fmt, nFaces,
		0, stretch_amnt, width, height, gutter, adj.get(), nullptr, nullptr, nullptr,
		DirectX::UVATLAS_DEFAULT_CALLBACK_FREQUENCY,
		//DirectX::UVATLAS_DEFAULT, vb, ib, nullptr, &remap);
		DirectX::UVATLAS_GEODESIC_FAST, vb, ib, nullptr, &remap);

	if (FAILED(result))
	{
		std::cout << "UV GENERATION FAILED! " << result << " (" << (result - INT_MIN) << ")" << std::endl;
		return false;
	}

	//std::cout << "Step one success!" << std::endl;

	auto newIB = reinterpret_cast<const int*>(ib.data());

#if VV_MESH_DEBUG_UV_GENERATION

	auto indices_new_size = ib.size() / sizeof(int);
	std::cout << "New indicies size: " << indices_new_size << std::endl;
	std::cout << "Old indicies size: " << triangles_vertices.size() << std::endl;

	for (int i = 0; i < indices_new_size; ++i)
	{
		std::cout << "INDICES " << i << ": " << triangles_vertices[i] << " -> " << newIB[i] << " (" << remap[newIB[i]] << ")" << std::endl;
	}

	std::cout << "Vertices size: " << vertices.size() << std::endl;
	std::cout << "Remap size: " << remap.size() << std::endl;

	for (int i = 0; i < vb.size(); ++i)
	{
		std::cout << "VERTICES " << i << " - " << remap[i] << ", x: " << (float)vertices[3 * remap[i]] << ", " << vb[i].pos.x << " (" << vb[remap[i]].pos.x << ")" << std::endl;
		std::cout << "VERTICES " << i << " - " << remap[i] << ", y: " << (float)vertices[3 * remap[i] + 1] << ", " << vb[i].pos.y << " (" << vb[remap[i]].pos.y << ")" << std::endl;
		std::cout << "VERTICES " << i << " - " << remap[i] << ", z: " << (float)vertices[3 * remap[i] + 2] << ", " << vb[i].pos.z << " (" << vb[remap[i]].pos.z << ")" << std::endl;
	}
#endif

	uvs.elements.resize(vb.size());
	for (int i = 0; i < vb.size(); ++i)
	{
		uvs.elements[i] = Eigen::Vector2d(vb[i].uv.x, vb[i].uv.y);
	}

	uvs.indices.resize(vertices.indices.size());

	for (int i = 0; i < uvs.indices.size(); ++i)
	{
		uvs.indices[i] = Eigen::Vector3i(newIB[3 * i], newIB[3 * i + 1], newIB[3 * i + 2]);
	}

	return true;
}

void VV_Mesh::SubdivideMesh(int iterations)
{
	for (int i = 0; i < iterations; ++i)
	{
		vertices.SubdivideAttributeAndReturnMidpoints();

		if (uvs.indices.size() > 0)
		{
			uvs.SubdivideAttributeAndReturnMidpoints();
		}

		if (normals.indices.size() > 0)
		{
			size_t initial_normals_size = normals.elements.size();
			normals.SubdivideAttributeAndReturnMidpoints();

			for (size_t i = initial_normals_size; i < normals.elements.size(); ++i)
			{
				normals.elements[i] = normals.elements[i].normalized();
			}
		}
	}
}

std::shared_ptr<std::vector<std::pair<size_t, std::vector<std::tuple<size_t, size_t, std::unordered_set<size_t>>>>>> VV_Mesh::SubdivideMeshAndGetAdjacencies(int iterations)
{
	auto to_return = std::make_shared<std::vector<std::pair<size_t, std::vector<std::tuple<size_t, size_t, std::unordered_set<size_t>>>>>>();
	to_return->resize(iterations);

	size_t start_index;

	for (int i = 0; i < iterations; ++i)
	{
		start_index = vertices.elements.size();
		(*to_return)[i].first = start_index;

		size_t initial_triangle_size = vertices.indices.size();
		auto midpoints = vertices.SubdivideAttributeAndReturnMidpoints();

		Eigen::Vector3i* tri;
		size_t divided_triangles_start_index;
		size_t vert_ind;

		(*to_return)[i].second.resize(vertices.elements.size() - start_index);

		for (size_t j = 0; j < initial_triangle_size; ++j)
		{
			tri = &vertices.indices[j];
			divided_triangles_start_index = initial_triangle_size + 3 * j;

			for (size_t k = 0; k < 3; ++k)
			{
				vert_ind = (*tri)[k];

				//Abusing the structure created from the attribute subdivision - the original triangle
				//is always replaced by one created from the 3 midpoints from subdivision.
				std::get<2>((*to_return)[i].second[vert_ind - start_index]).insert((*tri)[0]);
				std::get<2>((*to_return)[i].second[vert_ind - start_index]).insert((*tri)[1]);
				std::get<2>((*to_return)[i].second[vert_ind - start_index]).insert((*tri)[2]);

				//More abusing the structure - the 3 other triangles produced by subdivision always have 
				//the original triangle vertex as the first element.
				std::get<2>((*to_return)[i].second[vert_ind - start_index]).insert(vertices.indices[divided_triangles_start_index].x());
				std::get<2>((*to_return)[i].second[vert_ind - start_index]).insert(vertices.indices[divided_triangles_start_index + 1].x());
				std::get<2>((*to_return)[i].second[vert_ind - start_index]).insert(vertices.indices[divided_triangles_start_index + 2].x());
			}
		}

		size_t mid_index;
		size_t second_parent_index;

		for (size_t j = 0; j < midpoints->size(); ++j)
		{
			//The first entry in the midpoints always connects to the vertex itself, and as such can be safely ignored.
			for (size_t k = 1; k < (*midpoints)[j].size(); ++k)
			{
				//std::cout << "(" << j << ", " << k << ")" << std::endl;

				mid_index = (*midpoints)[j][k].second;
				second_parent_index = (*midpoints)[j][k].first;

				//Store the parents of a midpoint.
				std::get<0>((*to_return)[i].second[mid_index - start_index]) = j;
				std::get<1>((*to_return)[i].second[mid_index - start_index]) = second_parent_index;

				//The operation in the previous loop does not check if the index of the midpoint is contained in its adjacency.
				//This is a small clean-up operation to remove that.
				std::get<2>((*to_return)[i].second[mid_index - start_index]).erase(mid_index);
			}
		}

		if (uvs.indices.size() > 0)
		{
			uvs.SubdivideAttributeAndReturnMidpoints();
		}

		if (normals.indices.size() > 0)
		{
			size_t initial_normals_size = normals.elements.size();
			normals.SubdivideAttributeAndReturnMidpoints();

			for (size_t i = initial_normals_size; i < normals.elements.size(); ++i)
			{
				normals.elements[i] = normals.elements[i].normalized();
			}
		}
	}

	return to_return;
}

//Deemed unnecessary
void VV_Mesh::SelectivelySubdivideMesh(std::vector<unsigned char> &subdivision_counts)
{

}

void VV_Mesh::CopyFromMesh(VV_Mesh& other, int ignore_flags)
{
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_VERTICES))
	{
		vertices.CopyFrom(other.vertices);
	}
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_UVS))
	{
		uvs.CopyFrom(other.uvs);
	}
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_NORMALS))
	{
		normals.CopyFrom(other.normals);
	}

	//if (sm.num_vertices() > 0)
	//{
	//	sm = other.sm;
	//}
}

std::shared_ptr<VV_Mesh> VV_Mesh::GetCopy(int ignore_flags)
{
	auto new_mesh = std::make_shared<VV_Mesh>();

	new_mesh->CopyFromMesh(*this, ignore_flags);

	return new_mesh;
}

std::shared_ptr<VV_Mesh> VV_Mesh::ExtractSubmesh(std::vector<size_t>& target_indices, int ignore_flags)
{
	auto to_return = std::make_shared<VV_Mesh>();
	
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_VERTICES))
	{
		to_return->vertices.CopySubattribute(vertices, target_indices);
	}
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_UVS))
	{
		to_return->uvs.CopySubattribute(uvs, target_indices);
	}
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_NORMALS))
	{
		to_return->normals.CopySubattribute(normals, target_indices);
	}

	return to_return;
}

std::shared_ptr<draco::Mesh> VV_Mesh::ToDracoMesh()
{
	auto to_return = std::make_shared<draco::Mesh>();

	to_return->SetNumFaces(vertices.indices.size());

	to_return->set_num_points(vertices.indices.size() * 3);

	AddDracoAttribute(to_return, draco::GeometryAttribute::POSITION, vertices);
	AddDracoAttribute(to_return, draco::GeometryAttribute::TEX_COORD, uvs);
	AddDracoAttribute(to_return, draco::GeometryAttribute::NORMAL, normals);

	size_t face_count = vertices.indices.size();

	draco::Mesh::Face face;
	for (draco::FaceIndex i(0); i < face_count; ++i) {
		for (int c = 0; c < 3; ++c) {
			face[c] = 3 * i.value() + c;
		}
		to_return->SetFace(i, face);
	}

#ifdef DRACO_ATTRIBUTE_VALUES_DEDUPLICATION_SUPPORTED
	//to_return->DeduplicateAttributeValues();
#endif
#ifdef DRACO_ATTRIBUTE_INDICES_DEDUPLICATION_SUPPORTED
	//to_return->DeduplicatePointIds();
#endif

	return to_return;
}

void VV_Mesh::FromDracoMesh(draco::Mesh& mesh)
{
	Clear();

	RetrieveDracoAttribute(mesh, draco::GeometryAttribute::POSITION, vertices);
	RetrieveDracoAttribute(mesh, draco::GeometryAttribute::TEX_COORD, uvs);
	RetrieveDracoAttribute(mesh, draco::GeometryAttribute::NORMAL, normals);
}

void VV_Mesh::ConcatenateMesh(VV_Mesh& other, Eigen::Vector3d offset, int ignore_flags)
{
	size_t default_add = other.vertices.indices.size();

	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_VERTICES))
	{
		ConcatenateAttribute<Eigen::Vector3d>(other.vertices, vertices, offset, default_add);
	}
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_UVS))
	{
		ConcatenateAttribute<Eigen::Vector2d>(other.uvs, uvs, Eigen::Vector2d::Zero(), default_add);
	}
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_NORMALS))
	{
		ConcatenateAttribute<Eigen::Vector3d>(other.normals, normals, Eigen::Vector3d::Zero(), default_add);
	}
}

void VV_Mesh::ClearUnreferencedElements()
{
	vertices.ClearUnreferencedElements();
	uvs.ClearUnreferencedElements();
	normals.ClearUnreferencedElements();
}

void VV_Mesh::Clear(int ignore_flags)
{
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_VERTICES))
	{
		vertices.Clear();
	}
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_UVS))
	{
		uvs.Clear();
	}
	if (!(ignore_flags & VV_Mesh_Ignore_Flags::IGNORE_NORMALS))
	{
		normals.Clear();
	}
}

void VV_Mesh::FindClosestPointGreedy(Eigen::Vector3d point, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords)
{
	double closest_scalar = DBL_MAX;
	Eigen::Vector3d closest_vector;
	Eigen::Vector3d new_vector;

	Eigen::Vector3i* ind; 

	Eigen::Vector3d* p0;
	Eigen::Vector3d* p1;
	Eigen::Vector3d* p2;

	Eigen::Vector3d p01;
	Eigen::Vector3d p12;
	Eigen::Vector3d p20;

	Eigen::Vector3d triangle_normal; 

	for (size_t i = 0; i < vertices.indices.size(); ++i)
	{
		ind = &(vertices.indices[i]);

		p0 = &vertices.elements[ind->x()];
		p1 = &vertices.elements[ind->y()];
		p2 = &vertices.elements[ind->z()];

		p01 = (*p0) - (*p1);
		p12 = (*p1) - (*p2);
		p20 = (*p2) - (*p0);

		triangle_normal = Eigen::Vector3d(
			p01.y() * p20.z() - p01.z() * p20.y(),
			p01.z() * p20.x() - p01.x() * p20.z(),
			p01.x() * p20.y() - p01.y() * p20.x()
		);

		if (triangle_normal.squaredNorm() <= 0)
		{
			continue;
		}

		new_vector = VectorDistanceToTriangle(*p0, *p1, *p2, p01, p12, p20, point, triangle_normal);

		if (new_vector.squaredNorm() < closest_scalar)
		{
			out_triangle_index = i;
			closest_vector = new_vector;
			closest_scalar = closest_vector.squaredNorm();
		}
	}

	//std::cout << closest_vector.transpose() << std::endl;

	Eigen::Vector3d barycentric_input = point - closest_vector;

	if (out_triangle_index >= vertices.indices.size())
	{
		std::cout << "OH NO! index is " << out_triangle_index << std::endl;
	}

	ind = &(vertices.indices[out_triangle_index]);

	p0 = &vertices.elements[ind->x()];
	p1 = &vertices.elements[ind->y()];
	p2 = &vertices.elements[ind->z()];

	GetBarycentricCoordinatesOfTriangle(barycentric_input, *p0, *p1, *p2, barycentric_coords);
}

void VV_Mesh::FindClosestPointAmongList(std::vector<size_t> &to_check, Eigen::Vector3d point, size_t& out_triangle_index, Eigen::Vector3d& barycentric_coords)
{
	double closest_scalar = DBL_MAX;
	Eigen::Vector3d closest_vector;
	Eigen::Vector3d new_vector;

	Eigen::Vector3i ind;

	Eigen::Vector3d* p0;
	Eigen::Vector3d* p1;
	Eigen::Vector3d* p2;

	Eigen::Vector3d p01;
	Eigen::Vector3d p12;
	Eigen::Vector3d p20;

	Eigen::Vector3d triangle_normal;

	for (size_t i = 0; i < to_check.size(); ++i)
	{
		ind = vertices.indices[to_check[i]];

		p0 = &vertices.elements[ind.x()];
		p1 = &vertices.elements[ind.y()];
		p2 = &vertices.elements[ind.z()];

		p01 = (*p0) - (*p1);
		p12 = (*p1) - (*p2);
		p20 = (*p2) - (*p0);

		triangle_normal = Eigen::Vector3d(
			p01.y() * p20.z() - p01.z() * p20.y(),
			p01.z() * p20.x() - p01.x() * p20.z(),
			p01.x() * p20.y() - p01.y() * p20.x()
		);

		if (triangle_normal.squaredNorm() <= 0)
		{
			continue;
		}

		new_vector = VectorDistanceToTriangle(*p0, *p1, *p2, p01, p12, p20, point, triangle_normal);

		if (new_vector.squaredNorm() < closest_scalar)
		{
			out_triangle_index = to_check[i];
			closest_vector = new_vector;
			closest_scalar = closest_vector.squaredNorm();
		}
	}

	Eigen::Vector3d barycentric_input = closest_vector + point;

	ind = vertices.indices[out_triangle_index];

	p0 = &vertices.elements[ind.x()];
	p1 = &vertices.elements[ind.y()];
	p2 = &vertices.elements[ind.z()];

	GetBarycentricCoordinatesOfTriangle(barycentric_input, *p0, *p1, *p2, barycentric_coords);
}

void VV_Mesh::WriteVertices(std::ofstream& savefile)
{
	int v_count = vertices.elements.size();

	std::string to_write;

	for (int i = 0; i < v_count; ++i)
	{
		to_write = "v " +
			std::to_string(vertices.elements[i].x()) + " " +
			std::to_string(vertices.elements[i].y()) + " " +
			std::to_string(vertices.elements[i].z()) + "\n";

		savefile.write(to_write.c_str(), to_write.size());
	}
}

void VV_Mesh::WriteUVs(std::ofstream& savefile)
{
	int vt_count = uvs.elements.size();

	std::string to_write;

	for (int i = 0; i < vt_count; ++i)
	{
		to_write = "vt " +
			std::to_string(uvs.elements[i].x()) + " " +
			std::to_string(uvs.elements[i].y()) + "\n";

		savefile.write(to_write.c_str(), to_write.size());
	}
}

void VV_Mesh::WriteNormals(std::ofstream& savefile)
{
	int vn_count = normals.elements.size();

	std::string to_write;

	for (int i = 0; i < vn_count; ++i)
	{
		to_write = "vn " +
			std::to_string(normals.elements[i].x()) + " " +
			std::to_string(normals.elements[i].y()) + " " +
			std::to_string(normals.elements[i].z()) + "\n";

		savefile.write(to_write.c_str(), to_write.size());
	}
}

void VV_Mesh::WriteTriangles(std::ofstream& savefile)
{
	std::vector<bool> validations(3, false);

	validations[0] = (vertices.indices.size() > 0);
	validations[1] = (uvs.indices.size() > 0);
	validations[2] = (normals.indices.size() > 0);

	std::string to_write;

	int elem_max = 1;

	if (!validations[0])
	{
		std::cout << "ERROR - No triangle vertices!" << std::endl;
		return;
	}

	WriteVertices(savefile);

	if (validations[1])
	{
		elem_max = 2;
		WriteUVs(savefile);
	}

	if (validations[2])
	{
		elem_max = 3;
		WriteNormals(savefile);
	}

	//to_write = "f ";
	for (int i = 0; i < vertices.indices.size(); ++i)
	{
		to_write = "f " + std::to_string(vertices.indices[i].x() + 1);
		to_write += (validations[1] ? ("/" + std::to_string(uvs.indices[i].x() + 1)) : ((elem_max >= 2 ? "/" : "")));
		to_write += (validations[2] ? ("/" + std::to_string(normals.indices[i].x() + 1) + " ") : " ");

		to_write += std::to_string(vertices.indices[i].y() + 1);
		to_write += (validations[1] ? ("/" + std::to_string(uvs.indices[i].y() + 1)) : ((elem_max >= 2 ? "/" : "")));
		to_write += (validations[2] ? ("/" + std::to_string(normals.indices[i].y() + 1) + " ") : " ");

		to_write += std::to_string(vertices.indices[i].z() + 1);
		to_write += (validations[1] ? ("/" + std::to_string(uvs.indices[i].z() + 1)) : ((elem_max >= 2 ? "/" : "")));
		to_write += (validations[2] ? ("/" + std::to_string(normals.indices[i].z() + 1) + "\n") : "\n");

		savefile.write(to_write.c_str(), to_write.size());
	}

	savefile.close();
}

void VV_Mesh::DebugIrregularities(double very_far_away_threshold)
{
	size_t negative_index_count = 0;
	size_t oob_index_count = 0;
	size_t very_far_count = 0;

	for (size_t i = 0; i < vertices.indices.size(); ++i)
	{
		for (size_t j = 0; j < 3; ++j)
		{
			if (vertices.indices[i][j] < 0)
			{
				++negative_index_count;
			}

			if (vertices.indices[i][j] >= vertices.elements.size())
			{
				++oob_index_count;
			}
		}
	}

	for (size_t i = 0; i < vertices.elements.size(); ++i)
	{
		for (size_t j = 0; j < 3; ++j)
		{
			if (abs(vertices.elements[i][j]) > very_far_away_threshold)
			{
				++very_far_count;
			}
		}
	}

	std::cout << vertices.elements.size() << ", " << vertices.indices.size() << std::endl;

	std::cout << "Negative indices: " << negative_index_count << std::endl;
	std::cout << "OOB indices: " << oob_index_count << std::endl;
	std::cout << "Very far away vertices (>" << very_far_away_threshold << "): " << very_far_count << std::endl;
}

size_t VV_Mesh::FindNonManifoldEdges()
{
	size_t to_return = 0;

	//std::unordered_set<std::pair<size_t, size_t>> single_edges;
	std::vector<std::unordered_set<size_t>> connections;
	connections.resize(vertices.elements.size());

	size_t v0;
	size_t v1;

	for (size_t i = 0; i < vertices.indices.size(); ++i)
	{
		for (size_t j = 0; j < 3; ++j)
		{
			size_t j_1 = (j + 1) * (j < 2);

			v0 = vertices.indices[i][j];
			v1 = vertices.indices[i][j_1];

			if (connections[v0].contains(v1))
			{
				std::cout << "DUPLICATE EDGE: " << v0 << ", " << v1 << std::endl;
				++to_return;
			}
			else
			{
				connections[v0].insert(v1);
			}
		}
	}

	return to_return;
}
