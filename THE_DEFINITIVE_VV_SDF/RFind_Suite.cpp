#include "RFind_Suite.h"

void RFind_Suite::EmptyRFindTest()
{
	auto pos = test_string.rfind("", 0);

	std::cout << pos << ", " << std::string::npos << std::endl;

	pos = test_string.rfind("NotPresent", 0);

	std::cout << pos << ", " << std::string::npos << std::endl;
}

void RFind_Suite::run(int argc, char** argv)
{
	EmptyRFindTest();
}
