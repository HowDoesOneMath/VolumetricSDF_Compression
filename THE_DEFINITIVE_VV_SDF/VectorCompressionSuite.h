#pragma once

#include "TestSuite.h"

#include "BasicGeometry.h"
#include <iostream>

class VectorCompressionSuite : public TestSuite
{
	void TestVectorTruncation();
	void TestMatrixTruncation();
public:
	void run(int argc, char** argv);
};