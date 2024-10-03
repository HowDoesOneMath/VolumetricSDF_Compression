#include "XOR_Mask_Suite.h"

void XOR_Mask_Suite::XOR_Test_Mask()
{
	unsigned char mask = ~(1 << 7);

	unsigned char value = 255;

	auto masked = value & mask;

	std::cout << "Unmasked: " << (int)value << " Mask: " << (int)mask << ", Masked: " << (int)(value & mask) << std::endl;
}

void XOR_Mask_Suite::run(int argc, char** argv)
{
	XOR_Test_Mask();
}
