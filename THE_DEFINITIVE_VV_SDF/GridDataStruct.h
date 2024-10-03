#pragma once
#include "VV_SaveFileBuffer.h"

struct GridDataStruct
{
	GridDataStruct()
	{
		Clear();
	}

	GridDataStruct(size_t dim_x, size_t dim_y, size_t dim_z, double center_x, double center_y, double center_z, double unit_length)
	{
		this->dim_x = dim_x;
		this->dim_y = dim_y;
		this->dim_z = dim_z;

		this->center_x = center_x;
		this->center_y = center_x;
		this->center_z = center_x;

		this->unit_length = unit_length;
	}

	void Clear()
	{
		dim_x = 0;
		dim_y = 0;
		dim_z = 0;

		center_x = 0;
		center_y = 0;
		center_z = 0;

		unit_length = 0;
	}

	void WriteToBuffer(VV_SaveFileBuffer &buffer)
	{
		buffer.WriteObjectToBuffer(dim_x);
		buffer.WriteObjectToBuffer(dim_y);
		buffer.WriteObjectToBuffer(dim_z);

		buffer.WriteObjectToBuffer(center_x);
		buffer.WriteObjectToBuffer(center_y);
		buffer.WriteObjectToBuffer(center_z);

		buffer.WriteObjectToBuffer(unit_length);
	}

	void ReadFromBuffer(VV_SaveFileBuffer& buffer)
	{
		buffer.ReadObjectFromBuffer(dim_x);
		buffer.ReadObjectFromBuffer(dim_y);
		buffer.ReadObjectFromBuffer(dim_z);

		buffer.ReadObjectFromBuffer(center_x);
		buffer.ReadObjectFromBuffer(center_y);
		buffer.ReadObjectFromBuffer(center_z);

		buffer.ReadObjectFromBuffer(unit_length);
	}

	size_t dim_x = 0;
	size_t dim_y = 0;
	size_t dim_z = 0;

	double center_x = 0;
	double center_y = 0;
	double center_z = 0;

	double unit_length = 0;
};