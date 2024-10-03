#pragma once

//#if __INTELLISENSE__
//#undef __ARM_NEON
//#undef __ARM_NEON__
//#endif

#include <vector>
#include <Eigen/Core>

class AbstractAttribute
{
	size_t GetOrGenerateMidpointIndex(std::vector<std::pair<size_t, size_t>>& midpoints, size_t current_attribute, size_t to_find);

	void SubdivideTriangle(size_t index, size_t old_indices_size, std::vector<std::vector<std::pair<size_t, size_t>>>& midpoints);
protected:
	virtual size_t CreateNewMidpointElement(size_t current_attribute, size_t to_find) = 0;

	virtual void ClearElements() = 0;

public:

	std::vector<Eigen::Vector3i> indices;

	size_t ClearNegativeTriangles();

	//Eigen::
	virtual size_t GetElementCount() = 0;

	virtual size_t GetStride() = 0;

	virtual void ClearUnreferencedElements() = 0;

	virtual void CullAwkwardElements(double bad_edge_difference) = 0;

	/// <summary>
	/// Performs an in-place subdivision operation on the mesh attribute.
	/// Structure is produced as follows:
	/// 
	///            /\ A
	///           /  \
	///          / T2 \
	///     M_2 /______\ M_0
	///        /\      /\
	///       /  \ T1 /  \
	///      / T4 \  / T3 \
	///   C /______\/______\ B
	///            M_1
	/// 
	/// T1 = M_0, M_1, M_2 (located at original triangle index)
	/// 
	/// T2 = A, M_0, M_2 (located at 3 * i + initial_triangles)
	/// T3 = B, M_1, M_0 (located at 3 * i + initial_triangles + 1)
	/// T4 = C, M_2, M_1 (located at 3 * i + initial_triangles + 2)
	/// 
	/// </summary>
	/// <returns>An array; every index 'i' has its own array of connected vertices (of the un-subdivided mesh), 
	/// each dictating the opposing vertex index as well as the index of its midpoint. Only the least index of 
	/// every connected vertex pair contains this information (e.g. if edge (A, B) creates midpoint C, then array
	/// index A contains pair (B, C) - index B does not posses (A, C), as A is the lesser index).</returns>
	std::shared_ptr<std::vector<std::vector<std::pair<size_t, size_t>>>> SubdivideAttributeAndReturnMidpoints();

	virtual void Clear();

};