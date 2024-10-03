#include "MinMaxSuite.h"

void MinMaxSuite::TestMax()
{
	std::cout << "Max of " << n1 << " and " << n2 << ": " << std::max(n1, n2) << std::endl;
}

void MinMaxSuite::run(int argc, char** argv)
{
	TestMax();
}
