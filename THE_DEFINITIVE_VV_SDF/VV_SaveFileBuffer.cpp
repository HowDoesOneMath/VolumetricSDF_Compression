#include "VV_SaveFileBuffer.h"

bool VV_SaveFileBuffer::OpenWriteBuffer(std::string filename)
{
	if (write_buffer.is_open())
	{
		return false;
	}

	write_buffer.open(filename, std::ios::binary | std::ios::out);

	if (!write_buffer.is_open())
	{
		return false;
	}

	return true;
}

bool VV_SaveFileBuffer::OpenWriteBufferNonTruncate(std::string filename)
{
	if (write_buffer.is_open())
	{
		return false;
	}

	write_buffer.open(filename, std::ios::binary | std::ios::out | std::ios::in);

	if (!write_buffer.is_open())
	{
		return false;
	}

	return true;
}

bool VV_SaveFileBuffer::OpenReadBuffer(std::string filename)
{
	if (read_buffer.is_open())
	{
		return false;
	}

	read_buffer.open(filename, std::ios::binary | std::ios::in);

	if (!read_buffer.is_open())
	{
		return false;
	}

	return true;
}

bool VV_SaveFileBuffer::CloseWriteBuffer()
{
	if (!write_buffer.is_open())
	{
		return false;
	}

	write_buffer.close();

	return true;
}

bool VV_SaveFileBuffer::CloseReadBuffer()
{
	if (!read_buffer.is_open())
	{
		return false;
	}

	read_buffer.close();

	return true;
}