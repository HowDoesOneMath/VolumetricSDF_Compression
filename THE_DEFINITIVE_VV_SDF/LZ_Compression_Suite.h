#pragma once

#include "LZ_Encoder.h"
#include "TestSuite.h"

class LZ_Compression_Suite : public TestSuite
{
	LZ_Encoder encoder;

	std::string preset_string = "This is a sentence that can be written, definitely one of the sentences of all time. Wouldn't you agree that of all the sentences, this is one of them?";
	void TestWithPresetString();
public:
	void run(int argc, char** argv);
};