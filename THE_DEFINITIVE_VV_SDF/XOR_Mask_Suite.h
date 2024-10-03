#pragma once

#include "TestSuite.h"

#include <iostream>

//This didn't work like I thought it did, oh well
class XOR_Mask_Suite : public TestSuite
{
	void XOR_Test_Mask();
public:
	void run(int argc, char** argv);
};