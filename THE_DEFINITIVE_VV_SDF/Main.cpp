#include "SuiteMaster.h"

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
	std::cout << "Run? ";

	std::string check;

	std::cin >> check;

	std::cout << "Running..." << std::endl;
	
	SuiteMaster sm;

	sm.run(0, nullptr);

	std::cout << "\n\nFinished? ";

	std::cin >> check;

	return 0;
}