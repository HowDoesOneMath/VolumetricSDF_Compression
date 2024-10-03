#pragma once

#include <vector>
#include <iostream>

#include <unordered_map>

#include "AbstractAttribute.h"

//#include <Eigen/src/Core/Matrix.h>

template <typename T>
class VV_Mesh_Attribute : public AbstractAttribute
{
protected:
	size_t CreateNewMidpointElement(size_t current_attribute, size_t to_find);

	void ClearElements() { elements.clear(); }

public:
	std::vector<T> elements;

	size_t GetElementCount() { return elements.size(); }

	size_t GetStride() { return sizeof(T); }

	void ClearUnreferencedElements();

	void CullAwkwardElements(double bad_edge_length);

	void CullMassivelyDisplacedElements(std::vector<T>& displacements, double too_displaced_length);

	void CopyFrom(VV_Mesh_Attribute<T> &other) {
		elements = other.elements;
		indices = other.indices;
	}

	void CopySubattribute(VV_Mesh_Attribute<T>& other, std::vector<size_t> &target_indices)
	{
		if (other.indices.size() <= 0)
		{
			return;
		}

		indices.resize(target_indices.size());
		elements.clear();

		std::unordered_map<size_t, size_t> visited_elements;

		Eigen::Vector3i* tri;
		T* elem;

		for (int i = 0; i < target_indices.size(); ++i)
		{
			tri = &other.indices[target_indices[i]];

			for (int j = 0; j < 3; ++j)
			{
				if (!visited_elements.contains((size_t)(*tri)[j]))
				{
					visited_elements.insert(std::make_pair((size_t)(*tri)[j], elements.size()));
					elements.push_back(other.elements[(*tri)[j]]);
				}

				indices[i][j] = visited_elements[(size_t)(*tri)[j]];
			}
		}
	}

	void Concatenate(VV_Mesh_Attribute<T> &other, T offset) {
		size_t elem_count = elements.size();

		for (size_t i = 0; i < other.indices.size(); ++i)
		{
			indices.push_back(Eigen::Vector3i(
				other.indices[i].x() + elem_count,
				other.indices[i].y() + elem_count,
				other.indices[i].z() + elem_count
			));
		}

		for (size_t i = 0; i < other.elements.size(); ++i)
		{
			elements.push_back(other.elements[i] + offset);
		}
	}
};

template<typename T>
inline size_t VV_Mesh_Attribute<T>::CreateNewMidpointElement(size_t current_attribute, size_t to_find)
{
	size_t next_attribute = elements.size();

	elements.push_back((T)(elements[current_attribute] * 0.5 + elements[to_find] * 0.5));

	return next_attribute;
}

template<typename T>
inline void VV_Mesh_Attribute<T>::ClearUnreferencedElements()
{
	size_t precleared_elem_count = elements.size();

	std::vector<int> elem_ref_count;
	std::vector<int> elem_retarget;
	elem_ref_count.resize(elements.size());
	elem_retarget.resize(elements.size());

	for (size_t i = 0; i < precleared_elem_count; ++i)
	{
		//elem_ref_count[i] = 0;
		elem_retarget[i] = i;
	}

	size_t amount_unreferenced = precleared_elem_count;

	for (size_t i = 0; i < indices.size(); ++i)
	{
		amount_unreferenced -= (size_t)(elem_ref_count[indices[i][0]] == 0);
		++elem_ref_count[indices[i][0]];
		amount_unreferenced -= (size_t)(elem_ref_count[indices[i][1]] == 0);
		++elem_ref_count[indices[i][1]];
		amount_unreferenced -= (size_t)(elem_ref_count[indices[i][2]] == 0);
		++elem_ref_count[indices[i][2]];
	}

	for (size_t i = 0; i < elements.size(); ++i)
	{
		while ((i < elements.size()) && (elem_ref_count[i] == 0))
		{
			elem_retarget[elements.size() - 1] = i;
			elem_ref_count[i] = elem_ref_count[elements.size() - 1];

			elements[i] = elements.back();
			elements.pop_back();
		}
	}

	for (size_t i = 0; i < indices.size(); ++i)
	{
		indices[i][0] = elem_retarget[indices[i][0]];
		indices[i][1] = elem_retarget[indices[i][1]];
		indices[i][2] = elem_retarget[indices[i][2]];
	}
}

template<typename T>
inline void VV_Mesh_Attribute<T>::CullAwkwardElements(double bad_edge_difference)
{
	T* elem0;
	T* elem1;
	T* elem2;

	double edge_norm01;
	double edge_norm12;
	double edge_norm20;

	double max_dif;
	double min_dif;
	double edge_diff;

	int bad_tri_count = 0;

	for (size_t i = 0; i < indices.size(); ++i)
	{
		elem0 = &elements[indices[i][0]];
		elem1 = &elements[indices[i][1]];
		elem2 = &elements[indices[i][2]];

		edge_norm01 = (*elem0 - *elem1).norm();
		edge_norm12 = (*elem1 - *elem2).norm();
		edge_norm20 = (*elem2 - *elem0).norm();

		max_dif = std::max(edge_norm01, std::max(edge_norm12, edge_norm20));
		min_dif = std::min(edge_norm01, std::min(edge_norm12, edge_norm20));
		edge_diff = max_dif - min_dif;

		if (edge_diff > bad_edge_difference)
		{
			indices[i][0] = -1;
			indices[i][1] = -1;
			indices[i][2] = -1;

			++bad_tri_count;
		}
	}

	if (bad_tri_count > 0)
	{
		std::cout << "BAD TRIS: " << bad_tri_count << std::endl;
	}

	ClearNegativeTriangles();

	ClearUnreferencedElements();
}

template<typename T>
inline void VV_Mesh_Attribute<T>::CullMassivelyDisplacedElements(std::vector<T>& displacements, double too_displaced_length)
{
	int bad_tri_count = 0;

	double sqr_norm0;
	double sqr_norm1;
	double sqr_norm2;

	double sqr_thresh = too_displaced_length * too_displaced_length;
	bool is_too_displaced;

	for (size_t i = 0; i < indices.size(); ++i)
	{
		sqr_norm0 = displacements[indices[i][0]].squaredNorm();
		sqr_norm1 = displacements[indices[i][1]].squaredNorm();
		sqr_norm2 = displacements[indices[i][2]].squaredNorm();

		is_too_displaced = (sqr_norm0 > sqr_thresh) || (sqr_norm1 > sqr_thresh) || (sqr_norm2 > sqr_thresh);
		if (is_too_displaced)
		{
			indices[i][0] = -1;
			indices[i][1] = -1;
			indices[i][2] = -1;

			++bad_tri_count;
		}
	}


	if (bad_tri_count > 0)
	{
		std::cout << "BAD TRIS: " << bad_tri_count << std::endl;
	}

	ClearNegativeTriangles();

	ClearUnreferencedElements();
}
