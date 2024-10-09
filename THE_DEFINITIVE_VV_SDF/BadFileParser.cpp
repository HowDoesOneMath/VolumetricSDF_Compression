#include "BadFileParser.h"

bool BadFileParser::CheckFiles(std::string root_folder, SequenceFinderDetails to_check)
{
    if (!sf.FindFiles(root_folder, to_check))
    {
        std::cout << "Could not find files!" << std::endl;
        return false;
    }

    std::ifstream read_file;

    size_t bad_file_count = 0;

    for (size_t i = 0; i < sf.files[to_check.key].size(); ++i)
    {
        //std::cout << "Testing file " << sf.files[to_check.key][i] << std::endl;

        read_file.open(sf.files[to_check.key][i], std::ios::binary);

        if (!read_file.is_open())
        {
            std::cout << "COULD NOT OPEN " << sf.files[to_check.key][i] << std::endl;
            ++bad_file_count;
            continue;
        }

        bool bad_file = false;

        size_t current_line = 0;

        char test_character;
        while (read_file.get(test_character))
        {
            if (test_character < 0)
            {
                std::cout << "BAD FILE: " << sf.files[to_check.key][i] << std::endl;
                ++bad_file_count;
                break;
            }
        }

        read_file.close();
    }

    std::cout << "Bad files detected: " << bad_file_count << std::endl;

    return true;
}
