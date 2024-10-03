#pragma once

class UnionFindNode
{
public:
	size_t parent = 0;

	void MakeUnion(size_t other, UnionFindNode* data_array)
	{
		size_t my_parent = FindRoot(data_array);
		size_t other_parent = data_array[other].FindRoot(data_array);

		other_parent = my_parent;
	}

	size_t FindRoot(UnionFindNode* data_array)
	{
		if (parent == data_array[parent].parent)
		{
			return parent;
		}

		return data_array[parent].FindRoot(data_array);
	}
};