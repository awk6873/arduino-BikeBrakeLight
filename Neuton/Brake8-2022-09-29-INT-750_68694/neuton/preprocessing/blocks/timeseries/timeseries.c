/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "timeseries.h"
#include <string.h>

void NeutonPreprocessingBlockTimeseriesProcess(
	neuton_preprocessing_block_timeseries_instance *instance, const void *data, uint32_t elementsCount)
{
	if (!data || !elementsCount)
		return;

	const uint32_t hop = 
		((instance->windowHop == 0) || (instance->windowHop > instance->windowSize)) ? 
		instance->windowSize : instance->windowHop;
	
	const uint32_t tailSize = instance->windowSize - hop;
	uint8_t* dataFrom = (uint8_t*)data;
	uint8_t* dataTo = (uint8_t*)instance->windowBuffer;
	uint32_t offset = 0;

	while (elementsCount)
	{
		uint32_t nCopied = instance->windowSize - instance->currentFill;
		if (nCopied > elementsCount)
			nCopied = elementsCount;

		memcpy(
			&dataTo[instance->currentFill * instance->dataStride],
			&dataFrom[offset * instance->dataStride],
			nCopied * instance->dataStride);

		instance->currentFill += nCopied;
		if (instance->currentFill >= instance->windowSize)
		{
			if (instance->onWindow)
				instance->onWindow(instance, instance->windowBuffer);

			if (tailSize)
				memmove(dataTo, &dataTo[hop * instance->dataStride], tailSize * instance->dataStride);

			instance->currentFill = tailSize;
		}

		offset += nCopied;
		elementsCount -= nCopied;
	}
}
