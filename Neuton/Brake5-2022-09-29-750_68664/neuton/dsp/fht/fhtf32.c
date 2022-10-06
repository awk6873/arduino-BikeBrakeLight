/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "fhtf32.h"

typedef float fft_fixed_t;

void NeutonDspFhtMakeF32(const neuton_dsp_fht_instance* instance)
{
	fft_fixed_t* data = instance->windowArray;
	const fft_fixed_t* tgTable = instance->tgTable;
	const fft_fixed_t* sinTable = instance->sinTable;
	
	for (uint16_t i = 0; i < instance->fhtWindowSize; ++i)
	{
		if (i < instance->reversedBitIndexArray[i])
		{
			float tmp_value = data[i];
			data[i] = data[instance->reversedBitIndexArray[i]];
			data[instance->reversedBitIndexArray[i]] = tmp_value;
		}
	}

	for (uint16_t iteration = 1; iteration <= instance->logN; ++iteration)
	{
		uint16_t b_size = (uint16_t)1U << iteration;
		uint16_t b_amount = instance->fhtWindowSize >> iteration;
		if (b_size > 4)
		{
			for (uint16_t b_start = 0; b_start < instance->fhtWindowSize; b_start += b_size)
			{
				uint16_t pos1 = b_start + (b_size >> 1) + 1;
				uint16_t pos2 = b_start + b_size - 1;
				for (uint16_t index = b_amount; pos1 != pos2; ++pos1, --pos2, index += b_amount)
				{
					float tmp_value = data[pos2] - data[pos1] * tgTable[index];
					data[pos1] = data[pos1] + tmp_value * sinTable[index];
					data[pos2] = data[pos1] * tgTable[index] - tmp_value;
				}
			}
		}

		for (uint16_t b_start = 0; b_start < instance->fhtWindowSize; b_start += b_size)
		{
			uint16_t pos1 = b_start;
			uint16_t pos2 = b_start + (b_size >> 1);
			for (; pos2 < b_start + b_size; ++pos1, ++pos2)
			{
				float tmp_value = data[pos1];
				data[pos1] = tmp_value + data[pos2];
				data[pos2] = tmp_value - data[pos2];
			}
		}
	}

	data[0] = data[0] * data[0];
	for (uint16_t i = 1; i < (instance->fhtWindowSize >> 1); ++i)
	{
		data[i] = (data[i] * data[i] + data[instance->fhtWindowSize - i] * data[instance->fhtWindowSize - i]) / 2;
	}
}
