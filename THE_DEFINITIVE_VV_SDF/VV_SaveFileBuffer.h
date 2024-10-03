#pragma once

#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Core>

class VV_SaveFileBuffer
{
	std::fstream read_buffer; //ifstream
	std::fstream write_buffer; //ofstream

public:
	bool OpenWriteBuffer(std::string filename);
	bool OpenWriteBufferNonTruncate(std::string filename);
	bool OpenReadBuffer(std::string filename);

	bool CloseWriteBuffer();
	bool CloseReadBuffer();

	std::streampos GetWriterLocation() { return write_buffer.tellp(); }
	std::streampos GetReaderLocation() { return read_buffer.tellg(); }

	void SetWriterLocation(size_t location) { write_buffer.seekp(location); }
	void SetReaderLocation(size_t location) { read_buffer.seekg(location); }

	void SetWriterToEOF() { write_buffer.seekp(0, std::ios::end); }
	void SetReaderToEOF() { read_buffer.seekg(0, std::ios::end); }

	template<typename T>
	size_t WriteObjectToBuffer(T &toWrite);

	template<typename T>
	size_t ReadObjectFromBuffer(T &toRead);

	template<typename T>
	size_t WriteVectorToBuffer(std::vector<T> &toWrite);

	template<typename T>
	size_t ReadVectorFromBuffer(std::vector<T>& toRead);

	template<typename T>
	size_t WriteArrayToBuffer(T* toWrite, size_t count);

	template<typename T>
	size_t WriteArrayToBuffer(const T* toWrite, size_t count);

	template<typename T>
	size_t ReadArrayFromBuffer(T* toRead, size_t count);

	template<typename T>
	size_t ReadArrayFromBuffer(const T* toRead, size_t count);

	template<typename T>
	size_t WriteMatrixToBuffer(Eigen::MatrixX<T>& mat, size_t r_start, size_t c_start, size_t r_end, size_t c_end);

	template<typename T>
	size_t WriteVectorToBuffer(Eigen::VectorX<T>& vec, size_t v_start, size_t v_end);

	template<typename T>
	size_t ReadMatrixFromBuffer(Eigen::MatrixX<T>& presized_mat, size_t r_start, size_t c_start, size_t r_end, size_t c_end);

	template<typename T>
	size_t ReadVectorFromBuffer(Eigen::VectorX<T>& presized_vec, size_t v_start, size_t v_end);
};

template<typename T>
inline size_t VV_SaveFileBuffer::WriteObjectToBuffer(T &toWrite)
{
	size_t write_size = sizeof(T);
	write_buffer.write(reinterpret_cast<char*>(&toWrite), write_size);
	return write_size;
}

template<typename T>
inline size_t VV_SaveFileBuffer::ReadObjectFromBuffer(T &toRead)
{
	size_t read_size = sizeof(T);
	read_buffer.read(reinterpret_cast<char*>(&toRead), read_size);
	return read_size;
}

template<typename T>
inline size_t VV_SaveFileBuffer::WriteVectorToBuffer(std::vector<T>& toWrite)
{
	size_t elem_count = toWrite.size(); 
	size_t write_size = sizeof(T) * elem_count;
	write_buffer.write(reinterpret_cast<char*>(&elem_count), sizeof(size_t));
	write_buffer.write(reinterpret_cast<char*>(toWrite.data()), write_size);
	return write_size + sizeof(size_t);
}

template<typename T>
inline size_t VV_SaveFileBuffer::ReadVectorFromBuffer(std::vector<T>& toRead)
{
	size_t elem_count;
	read_buffer.read(reinterpret_cast<char*>(&elem_count), sizeof(size_t));
	toRead.resize(elem_count);
	read_buffer.read(reinterpret_cast<char*>(toRead.data()), elem_count * sizeof(T));
	return elem_count * sizeof(T) + sizeof(size_t);
}

template<typename T>
inline size_t VV_SaveFileBuffer::WriteArrayToBuffer(T* toWrite, size_t count)
{
	write_buffer.write(reinterpret_cast<char*>(toWrite), count * sizeof(T));
	return count * sizeof(T);
}

template<typename T>
inline size_t VV_SaveFileBuffer::WriteArrayToBuffer(const T* toWrite, size_t count)
{
	write_buffer.write(reinterpret_cast<const char*>(toWrite), count * sizeof(T));
	return count * sizeof(T);
}

template<typename T>
inline size_t VV_SaveFileBuffer::ReadArrayFromBuffer(T* toRead, size_t count)
{
	read_buffer.read(reinterpret_cast<char*>(toRead), count * sizeof(T));
	return count * sizeof(T);
}

template<typename T>
inline size_t VV_SaveFileBuffer::ReadArrayFromBuffer(const T* toRead, size_t count)
{
	read_buffer.read(reinterpret_cast<const char*>(toRead), count * sizeof(T));
	return count * sizeof(T);
}

template<typename T>
inline size_t VV_SaveFileBuffer::WriteMatrixToBuffer(Eigen::MatrixX<T>& mat, size_t r_start, size_t c_start, size_t r_end, size_t c_end)
{
	size_t elem_count = (r_end - r_start) * (c_end - c_start);
	size_t byte_size = elem_count * sizeof(T);

	std::vector<T> data_buf(elem_count);
	size_t elem = 0;

	for (size_t r = r_start; r < r_end; ++r)
	{
		for (size_t c = c_start; c < c_end; ++c, ++elem)
		{
			data_buf[elem] = mat(r, c);
		}
	}

	write_buffer.write(reinterpret_cast<char*>(data_buf.data()), byte_size);
	return byte_size;
}

template<typename T>
inline size_t VV_SaveFileBuffer::WriteVectorToBuffer(Eigen::VectorX<T>& vec, size_t v_start, size_t v_end)
{
	size_t elem_count = v_end - v_start;
	size_t byte_size = elem_count * sizeof(T);

	std::vector<T> data_buf(elem_count);
	size_t elem = 0;

	for (size_t v = v_start; v < v_end; ++v, ++elem)
	{
		data_buf[elem] = vec(v);
	}

	write_buffer.write(reinterpret_cast<char*>(data_buf.data()), byte_size);
	return byte_size;
}

template<typename T>
inline size_t VV_SaveFileBuffer::ReadMatrixFromBuffer(Eigen::MatrixX<T>& presized_mat, size_t r_start, size_t c_start, size_t r_end, size_t c_end)
{
	size_t elem_count = (r_end - r_start) * (c_end - c_start);
	size_t byte_size = elem_count * sizeof(T);

	std::vector<T> data_buf(elem_count);
	size_t elem = 0;

	read_buffer.read(reinterpret_cast<char*>(data_buf.data()), byte_size);

	for (size_t r = r_start; r < r_end; ++r)
	{
		for (size_t c = c_start; c < c_end; ++c, ++elem)
		{
			presized_mat(r, c) = data_buf[elem];
		}
	}

	return byte_size;
}

template<typename T>
inline size_t VV_SaveFileBuffer::ReadVectorFromBuffer(Eigen::VectorX<T>& presized_vec, size_t v_start, size_t v_end)
{
	size_t elem_count = v_end - v_start;
	size_t byte_size = elem_count * sizeof(T);

	std::vector<T> data_buf(elem_count);
	size_t elem = 0;

	read_buffer.read(reinterpret_cast<char*>(data_buf.data()), byte_size);

	for (size_t v = v_start; v < v_end; ++v, ++elem)
	{
		presized_vec(v) = data_buf[elem];
	}

	return byte_size;
}
