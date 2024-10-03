#pragma once

#include "TestSuite.h"

#include <string>
#include <iostream>

class MinMaxSuite : public TestSuite
{
	size_t n1 = -1;
	size_t n2 = 1;

	void TestMax();
public:
	void run(int argc, char** argv);
};