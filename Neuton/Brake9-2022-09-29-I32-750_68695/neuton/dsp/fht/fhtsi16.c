/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "fhtsi16.h"

typedef int16_t fft_fixed_t;
typedef int32_t fft_fixed2_t;
typedef uint16_t fft_count_t;

#define FIXED_MULT(x, y) (((fft_fixed2_t)(x) * (y)) >> FRACT_BITS_15)
#define FIXED_DIV(x, y) (((fft_fixed2_t)(x) << FRACT_BITS_15) / (y))
#define GET_OVERFLOW_MARKER(x) (x > INT16_MAX || x < INT16_MIN)

uint16_t NeutonDspFhtMake16(const neuton_dsp_fht_instance* instance)
{
	fft_fixed_t* data = instance->windowArray;
	const fft_fixed_t* tgTable = instance->tgTable;
	const fft_fixed_t* sinTable = instance->sinTable;
	
	for (fft_count_t i = 0; i < instance->fhtWindowSize; ++i)
	{
		if (i < instance->reversedBitIndexArray[i])
		{
			fft_fixed_t tmp_value = data[i];
			data[i] = data[instance->reversedBitIndexArray[i]];
			data[instance->reversedBitIndexArray[i]] = tmp_value;
		}
	}

	uint16_t ex = 0;

	for (uint8_t iteration = 1; iteration <= instance->logN; ++iteration)
	{
		fft_count_t b_size = (fft_count_t)1U << iteration;
		fft_count_t b_amount = instance->fhtWindowSize >> iteration;
		fft_fixed2_t inc = 0;

		if (b_size > 4)
		{
			for (fft_count_t b_start = 0; b_start < instance->fhtWindowSize; b_start += b_size)
			{
				fft_count_t pos1 = b_start + (b_size >> 1) + 1;
				fft_count_t pos2 = b_start + b_size - 1;
				for (fft_count_t index = b_amount; pos1 != pos2; ++pos1, --pos2, index += b_amount)
				{
					fft_fixed2_t pos1_value = data[pos1];
					fft_fixed2_t pos2_value = data[pos2];
					fft_fixed2_t sin_value = sinTable[index];
					fft_fixed2_t tg_value = tgTable[index];

					fft_fixed2_t tmp_value = pos2_value - FIXED_MULT(pos1_value, tg_value);
					pos1_value = pos1_value + FIXED_MULT(tmp_value, sin_value);
					pos2_value = FIXED_MULT(pos1_value, tg_value) - tmp_value;

					if (inc == 0 && (GET_OVERFLOW_MARKER(pos1_value) || GET_OVERFLOW_MARKER(pos2_value)))
					{
						inc = 1;
						++ex;
						fft_count_t pos1r = b_start + (b_size >> 1) + 1;
						fft_count_t pos2r = b_start + b_size - 1;
						for (; pos1r < pos1; ++pos1r, --pos2r)
						{
							data[pos1r] >>= inc;
							data[pos2r] >>= inc;
						}
					}

					data[pos1] = (fft_fixed_t)(pos1_value >> inc);
					data[pos2] = (fft_fixed_t)(pos2_value >> inc);
				}

				if (inc == 1)
				{
					fft_count_t pos1r = b_start + (b_size >> 1) + 1;
					fft_count_t pos2r = b_start + b_size - 1;
					data[(pos1r + pos2r) / 2] >>= inc;
					for (pos1r = b_start; pos1r < b_start + (b_size >> 1) + 1; ++pos1r)
					{
						data[pos1r] >>= inc;
					}
				}
			}
			inc = 0;
		}

		for (fft_count_t b_start = 0; b_start < instance->fhtWindowSize; b_start += b_size)
		{
			fft_count_t pos1 = b_start;
			fft_count_t pos2 = b_start + (b_size >> 1);
			for (; pos2 < b_start + b_size; ++pos1, ++pos2)
			{
				fft_fixed2_t pos1_value = data[pos1];
				fft_fixed2_t pos2_value = data[pos2];

				fft_fixed2_t tmp_value = pos1_value;
				pos1_value = tmp_value + pos2_value;
				pos2_value = tmp_value - pos2_value;

				if (inc == 0 && (GET_OVERFLOW_MARKER(pos1_value) || GET_OVERFLOW_MARKER(pos2_value)))
				{
					inc = 1;
					++ex;
					fft_count_t pos1r = b_start;
					fft_count_t pos2r = b_start + (b_size >> 1);
					for (; pos1r < pos1; ++pos1r, ++pos2r)
					{
						data[pos1r] >>= inc;
						data[pos2r] >>= inc;
					}
					for (pos1r = 0; pos1r < b_start; ++pos1r)
					{
						data[pos1r] >>= inc;
					}
				}

				data[pos1] = (fft_fixed_t)(pos1_value >> inc);
				data[pos2] = (fft_fixed_t)(pos2_value >> inc);
			}
		}
	}

	data[0] = FIXED_MULT(data[0], data[0]);
	fft_fixed2_t inc = 0;
	if (data[0] & ((fft_fixed_t)1U << (FRACT_BITS_15 - 1)))
	{
		inc = 1;
		++ex;
	}
	else
	{
		data[0] <<= 1;
	}

	for (fft_count_t i = 1, ri = instance->fhtWindowSize - 1; i < (instance->fhtWindowSize >> 1); ++i, --ri)
	{
		fft_fixed2_t power_value = FIXED_MULT(data[i], data[i]) + FIXED_MULT(data[ri], data[ri]);
		if (inc == 0 && GET_OVERFLOW_MARKER(power_value))
		{
			inc = 1;
			++ex;
			for (fft_count_t j = 0; j < i; ++j)
				data[j] >>= 1;
		}
		data[i] = (fft_fixed_t)(power_value >> inc);
	}
	ex = FRACT_BITS_15 - ex;
	return ex;
}

