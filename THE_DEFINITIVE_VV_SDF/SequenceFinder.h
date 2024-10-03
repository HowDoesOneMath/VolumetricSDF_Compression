#pragma once

#include "AdditionalUtilities.h"

#include <vector>
#include <string>

//#include <unordered_map>
#include <map>

struct SequenceFinderDetails
{
	SequenceFinderDetails(std::string key, std::string file_ext, std::string file_tag)
	{
		this->key = key;
		this->file_ext = file_ext;
		this->file_tag = file_tag;
	}

	SequenceFinderDetails(std::string key, std::string file_ext)
	{
		this->key = key;
		this->file_ext = file_ext;
	}

	/// <summary>
	/// The key that will be used to access this data in its respective SequenceFinder, must be set
	/// </summary>
	std::string key;

	/// <summary>
	/// The file extension of the expected data, must be set
	/// </summary>
	std::string file_ext;

	/// <summary>
	/// Tag to filter data, can be set to "" for no tag
	/// </summary>
	std::string file_tag = "";
};

class SequenceFinder
{
public:
	std::map<std::string, std::vector<std::string>> files;

	std::shared_ptr<std::vector<std::string>> GetFileVector(std::string key);

	bool FindFiles(std::string input_folder, SequenceFinderDetails &details);
	bool FindFiles(std::string input_folder, std::vector<SequenceFinderDetails> &details);
	bool FindFiles(std::string input_folder, SequenceFinderDetails *details, int detail_count);
};