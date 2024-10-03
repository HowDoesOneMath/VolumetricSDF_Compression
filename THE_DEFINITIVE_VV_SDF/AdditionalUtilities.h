#pragma once

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <filesystem>
#include <fstream>

inline std::string DateTodayManual()
{
    return "6_28_2024";
}

inline void SplitString(std::string to_split, std::vector<std::string>& destination, std::string delims, std::string ignore_characters = "", bool cull_empty_strings = true)
{
    size_t iterator = 0;
    size_t loc = 0;

    for (int i = 0; i < ignore_characters.length(); ++i)
    {
        auto to_erase = to_split.find(ignore_characters[i]);

        while (to_erase != std::string::npos)
        {
            to_split.erase(to_erase, 1);

            to_erase = to_split.find(ignore_characters[i]);
        }
    }

    while (loc < to_split.size())
    {
        loc = to_split.find_first_of(delims, iterator);
        std::string temp_data = to_split.substr(iterator, loc - iterator);

        if (!cull_empty_strings || (temp_data.size() > 0))
        {
            destination.push_back(temp_data);
        }

        iterator = loc + 1;
    }
}

inline std::vector<std::string> GetDirectories(const std::string& s)
{
    std::vector<std::string> r;
    //std::file
    for (auto& p : std::filesystem::directory_iterator(s))
        if (p.is_directory())
            r.push_back(p.path().string());
    return r;
}

inline std::vector<std::string> GetFiles(const std::string& s)
{
    std::vector<std::string> r;
    
    for (auto& p : std::filesystem::directory_iterator(s))
        if (p.is_regular_file())
            r.push_back(p.path().string());
    return r;
}

inline std::vector<std::string> GetFilesWithTag(std::vector<std::string> f, const std::string& tag)
{
    std::vector<std::string> r;
    for (auto& p : f)
        if (p.rfind(tag) != std::string::npos)
            r.push_back(p);
    return r;
}

inline std::vector<std::string> GetFilesWithTags(std::vector<std::string> f, const std::vector<std::string>& tags)
{
    std::vector<std::string> r;
    for (auto& p : f)
        for (auto& t : tags)
            if (p.rfind(t) != std::string::npos)
            {
                r.push_back(p);
                break;
            }
    return r;
}

inline void DebugVector(std::vector<std::string>& vec)
{
    for (int i = 0; i < vec.size(); ++i)
    {
        std::cout << std::to_string(i) << ": " << vec[i] << std::endl;
    }
}

inline void WriteError(std::exception& e, std::string filename, std::string otherData = "")
{
    std::fstream emptyTXT;

    std::string what = e.what();
    what += "\n";

    emptyTXT.open(filename, std::ios_base::out);

    emptyTXT.write(what.c_str(), what.length());
    emptyTXT.write(otherData.c_str(), otherData.length());
    emptyTXT.close();
}

inline void Dbg(std::string msg)
{
    std::cout << msg << std::endl;
    system("pause");
}

inline std::string GetNumberFixedLength(int num, int length)
{
    std::string to_return;
    std::string iter_num = std::to_string(num);
    to_return.resize(length - iter_num.size(), '0');
    to_return.append(iter_num);

    return to_return;
}

inline std::string RemoveFileExtention(std::string filename)
{
    auto loc = filename.rfind('.');

    if (loc == std::string::npos)
    {
        return filename;
    }

    return filename.substr(0, loc);
}

inline std::string StringLowerCase(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return std::tolower(c); }
    );
    return s;
}

template<typename T>
inline size_t WriteObjectBytes(T& toWrite, char* output_buffer)
{
    size_t write_size = sizeof(T);
    memcpy(output_buffer, &toWrite, write_size);
    return write_size;
}

template<typename T>
size_t ReadObjectBytes(T& toRead, char* input_buffer)
{
    size_t read_size = sizeof(T);
    memcpy(&toRead, input_buffer, read_size);
    return read_size;
}

template<typename T>
size_t WriteVectorBytes(std::vector<T>& toWrite, char* presized_output_buffer)
{
    size_t elem_count = toWrite.size();
    size_t write_size = sizeof(T) * elem_count;
    memcpy(presized_output_buffer, &elem_count, sizeof(size_t));
    memcpy(presized_output_buffer + sizeof(size_t), toWrite.data(), write_size);
    return write_size + sizeof(size_t);
}

template<typename T>
size_t ReadVectorBytes(std::vector<T>& toRead, char* presized_input_buffer)
{
    size_t elem_count;
    memcpy(&elem_count, presized_input_buffer, sizeof(size_t));
    toRead.resize(elem_count);
    memcpy(toRead.data(), presized_input_buffer + sizeof(size_t), elem_count * sizeof(T));
    return elem_count * sizeof(T) + sizeof(size_t);
}

template<typename T>
inline size_t WriteObjectToBuffer(T& toWrite, std::ofstream write_buffer)
{
    size_t write_size = sizeof(T);
    write_buffer.write(reinterpret_cast<char*>(&toWrite), write_size);
    return write_size;
}

template<typename T>
inline size_t ReadObjectFromBuffer(T& toRead, std::ifstream read_buffer)
{
    size_t read_size = sizeof(T);
    read_buffer.read(reinterpret_cast<char*>(&toRead), read_size);
    return read_size;
}

template<typename T>
inline size_t WriteVectorToBuffer(std::vector<T>& toWrite, std::ofstream write_buffer)
{
    size_t elem_count = toWrite.size();
    size_t write_size = sizeof(T) * elem_count;
    write_buffer.write(reinterpret_cast<char*>(&elem_count), sizeof(size_t));
    write_buffer.write(reinterpret_cast<char*>(toWrite.data()), write_size);
    return write_size + sizeof(size_t);
}

template<typename T>
inline size_t ReadVectorFromBuffer(std::vector<T>& toRead, std::ifstream read_buffer)
{
    size_t elem_count;
    read_buffer.read(reinterpret_cast<char*>(&elem_count), sizeof(size_t));
    toRead.resize(elem_count);
    read_buffer.read(reinterpret_cast<char*>(toRead.data()), elem_count * sizeof(T));
    return elem_count * sizeof(T) + sizeof(size_t);
}