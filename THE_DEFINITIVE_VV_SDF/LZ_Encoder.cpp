#include "LZ_Encoder.h"

bool LZ_Encoder::CompressData(unsigned char* data, size_t data_size, std::vector<unsigned char>& output_vector)
{
    const size_t max_dst_size = LZ4_compressBound(data_size);

    output_vector.resize(max_dst_size);

    const size_t compressed_data_size = LZ4_compress_default((char*)data, (char*)output_vector.data(), data_size, max_dst_size);

    if (compressed_data_size <= 0)
    {
        std::cout << "ERROR! Compressed data size below 0!" << std::endl;
        return false;
    }

    output_vector.resize(compressed_data_size);

    return true;
}

bool LZ_Encoder::DecompressData(unsigned char* data, size_t data_size, std::vector<unsigned char>& presized_output_vector)
{
    const size_t decompressed_size = LZ4_decompress_safe((char*)data, (char*)presized_output_vector.data(), data_size, presized_output_vector.size());

    if (decompressed_size < 0)
    {
        std::cout << "ERROR! Decompressed data size below 0!" << std::endl;
        return false;
    }

    return true;
}

size_t LZ_Encoder::SaveToBuffer(VV_SaveFileBuffer& sfb, unsigned char* data, size_t data_size)
{
    const size_t max_dst_size = LZ4_compressBound(data_size);
    std::vector<unsigned char> output_vector(max_dst_size);

    const size_t compressed_data_size = LZ4_compress_default((char*)data, (char*)output_vector.data(), data_size, max_dst_size);

    if (compressed_data_size <= 0)
    {
        std::cout << "ERROR! Compressed data size below 0!" << std::endl;
        return 0;
    }

    output_vector.resize(compressed_data_size);
    
    sfb.WriteVectorToBuffer(output_vector);

    return compressed_data_size;
}

size_t LZ_Encoder::ReadFromBuffer(VV_SaveFileBuffer& sfb, unsigned char* presized_data, size_t data_size)
{
    std::vector<unsigned char> input_data;
    sfb.ReadVectorFromBuffer(input_data);

    const size_t decompressed_size = LZ4_decompress_safe((char*)input_data.data(), (char*)presized_data, input_data.size(), data_size);

    if (decompressed_size < 0)
    {
        std::cout << "ERROR! Decompressed data size below 0!" << std::endl;
        return 0;
    }

    return input_data.size();
}
