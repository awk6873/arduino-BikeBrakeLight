/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "normalize.h"

void NeutonPreprocessingBlockNormalize(const float *src, float *dst, size_t count, float min, float max)
{
	float diff = max - min;

	for (size_t i = 0; i < count; ++i)
	{
		float result = src[i] - min;

		if (diff)
			result /= diff;

		if (result > 1.0)
			result = 1.0;
		if (result < 0.0)
			result = 0.0;

		dst[i] = result;
	}
}
