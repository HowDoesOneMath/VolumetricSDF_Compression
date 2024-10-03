#include "SequenceFinder.h"

std::shared_ptr<std::vector<std::string>> SequenceFinder::GetFileVector(std::string key)
{
	return std::make_shared<std::vector<std::string>>(files[key]);
}

bool SequenceFinder::FindFiles(std::string input_folder, SequenceFinderDetails& details)
{
	std::vector<std::string> all_files;
	all_files = GetFiles(input_folder);

	for (int i = 0; i < all_files.size(); ++i)
	{
		auto file_end = StringLowerCase(all_files[i].substr(all_files[i].size() - 4, 4));
		if (file_end == details.file_ext)
		{
			if (all_files[i].rfind(details.file_tag) != std::string::npos)
			{
				files[details.key].push_back(all_files[i]);
			}
		}
	}

	if (files.size() <= 0)
	{
		std::cout << "No valid \"" << details.file_tag << "\" files at " << input_folder << "!" << std::endl;
		return false;
	}

	return true;
}

bool SequenceFinder::FindFiles(std::string input_folder, std::vector<SequenceFinderDetails>& details)
{
	std::vector<std::string> all_files;
	all_files = GetFiles(input_folder);

	for (int i = 0; i < all_files.size(); ++i)
	{
		for (int j = 0; j < details.size(); ++j)
		{
			auto file_end = StringLowerCase(all_files[i].substr(all_files[i].size() - 4, 4));
			if (file_end == details[j].file_ext)
			{
				if (all_files[i].rfind(details[j].file_tag) != std::string::npos)
				{
					files[details[j].key].push_back(all_files[i]);
				}
			}
		}
	}

	for (int j = 0; j < details.size(); ++j)
	{
		if (files[details[j].key].size() <= 0)
		{
			std::cout << "No valid \"" << details[j].file_tag << "\" files at " << input_folder << "!" << std::endl;
			return false;
		}
	}

	return true;
}

bool SequenceFinder::FindFiles(std::string input_folder, SequenceFinderDetails* details, int detail_count)
{
	std::vector<std::string> all_files;
	all_files = GetFiles(input_folder);

	for (int i = 0; i < all_files.size(); ++i)
	{
		for (int j = 0; j < detail_count; ++j)
		{
			auto file_end = StringLowerCase(all_files[i].substr(all_files[i].size() - 4, 4));
			if (file_end == details[j].file_ext)
			{
				if (all_files[i].rfind(details[j].file_tag) != std::string::npos)
				{
					files[details[j].key].push_back(all_files[i]);
				}
			}
		}
	}

	for (int j = 0; j < detail_count; ++j)
	{
		if (files[details[j].key].size() <= 0)
		{
			std::cout << "No valid \"" << details[j].file_tag << "\" files at " << input_folder << "!" << std::endl;
			return false;
		}
	}

	return true;
}
