#pragma once

#include "TestSuite.h"

#include "RunLengthEncoder.h"
#include <string>
#include <iostream>

class RunLengthSuite : public TestSuite
{
	RunLengthEncoder rle;
	//std::string test_data = "aaaaaaaaaaaaaaaaaaaaaaabcddddeeffffffffffgghhhhhhhhhhhhhhhhhh";
	std::string test_data = "aaaaaaaaaaaaaaaaaaaaaaabbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

public:
	void run(int argc, char** argv);
};