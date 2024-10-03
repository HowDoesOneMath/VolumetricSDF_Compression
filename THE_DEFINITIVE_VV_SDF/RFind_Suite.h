#pragma once

#include "TestSuite.h"

#include <string>
#include <iostream>

//Exists to check if an empty string is considered 'present' in a full string. The check succeeded.
class RFind_Suite : public TestSuite
{
	std::string test_string = "ThisIsAString";

	void EmptyRFindTest();
public:
	void run(int argc, char** argv);
};