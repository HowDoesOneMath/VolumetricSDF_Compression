#pragma once

#include <stdlib.h>
#include <time.h>

class RandomWrapper
{
	static bool srand_initialized;
public:
	RandomWrapper() {
		if (srand_initialized)
		{
			return;
		}

		srand(time(0));
		srand_initialized = true;
	}

	//Lower is inclusive, higher is exclusive
	inline int InRange(int lower, int higher) {
		return rand() % (higher - lower) + lower;
	}
};