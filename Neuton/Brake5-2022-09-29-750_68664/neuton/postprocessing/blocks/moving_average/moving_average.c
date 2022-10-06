/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "moving_average.h"

int8_t NeutonPostprocessingBlockMovingAverageProcess(
	neuton_postprocessing_block_moving_average_instance *instance,
	const float *data, float **output, uint16_t *index)
{
	if (!data || !output)
		return 1;

	float *currentValues = &instance->windowBuffer[instance->currentIndex * instance->elementsCount];
	for (uint16_t i = 0; i < instance->elementsCount; ++i)
	{
		currentValues[i] = data[i];
		instance->averages[i] = 0;
	}

	if (!instance->isInitialized && (instance->currentIndex == (instance->windowSize - 1)))
	{
		instance->isInitialized = 1;
	}

	instance->currentIndex = (instance->currentIndex + 1) % instance->windowSize;

	if (!instance->isInitialized)
	{
		return 1;
	}

	if (instance->suppressionCurrentCounter)
	{
		if (--instance->suppressionCurrentCounter)
		{
			return 1;
		}
	}

	for (uint32_t n = 0; n < instance->windowSize; ++n)
	{
		float *values = &instance->windowBuffer[n * instance->elementsCount];
		for (uint32_t i = 0; i < instance->elementsCount; ++i)
		{
			instance->averages[i] += values[i];
		}
	}

	for (uint32_t i = 0; i < instance->elementsCount; ++i)
	{
		instance->averages[i] /= instance->windowSize;
	}

	int32_t maxIndex = -1;
	float maxValue = 0;
	for (int32_t i = 0; i < instance->elementsCount; ++i)
	{
		if ((maxIndex < 0) || (maxValue < instance->averages[i]))
		{
			maxIndex = i;
			maxValue = instance->averages[i];
		}
	}

	if (maxValue < instance->threshold)
	{
		return 1;
	}

	*output = instance->averages;
	if (index)
	{
		*index = maxIndex;
	}

	instance->suppressionCurrentCounter = instance->suppressionCount;

	return 0;
}
