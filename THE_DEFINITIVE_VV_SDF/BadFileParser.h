#pragma once

#include <fstream>
#include <string>
#include <iostream>

#include "SequenceFinder.h"

class BadFileParser
{
	SequenceFinder sf;

public:
	bool CheckFiles(std::string root_folder, SequenceFinderDetails to_check);
};