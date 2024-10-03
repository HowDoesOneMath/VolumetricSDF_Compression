#pragma once

#include "TestSuite.h"

#include <iostream>
#include <Eigen/Core>

//This didn't work like I thought it did, oh well
class EigenSuite : public TestSuite
{
	void TestComparitorAcrossCoefficients();
public:
	void run(int argc, char** argv);
};