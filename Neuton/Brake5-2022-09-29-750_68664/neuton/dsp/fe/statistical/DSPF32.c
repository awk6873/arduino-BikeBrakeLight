/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "DSPF32.h"
#include "../../fht/fhtf32.h"

#include <stdlib.h>
#include <math.h>

/**
 * \brief Common Data buffer
 */
typedef struct
{
	const input_dsp_f32_t* Data;

	output_dsp_f32_t Min;

	output_dsp_f32_t Max;

	output_dsp_f32_t Moments[3];

	output_dsp_f32_t Mean;

	output_dsp_f32_t AmpP2PHF;

	output_dsp_f32_t AmpP2PLF;

	output_dsp_f32_t FhtPowerArray[FHT_RESULTS_NUMBER];
	
	uint16_t FhtFreqArray[FHT_RESULTS_NUMBER];

	size_t Size;

	uint16_t MeanCrossing;

	uint16_t NegMeanCrossing;

	uint16_t PosMeanCrossing;

} TDataBufferF32;

static TDataBufferF32 DataBufferF32;

///
/// \brief Calcs peak values for Fht
/// \param inputData - Data to calc in (element tipe is input_t)
/// \param dataSize - Size of the buffer in elements.
/// \param powerEF - Output buffer for power. Size of 3 elements
/// \param freq - Output buffer for indexes for each power. Size of 3 elements
///
void CalcSpectrumPeaksF32(input_dsp_f32_t* inputData, uint16_t dataSize, output_dsp_f32_t* powerEF, uint16_t* freq)
{
	inputData[0] = 0;  // remove constant component of the signal

	/* Algorithm II */
	for (size_t i = 0; i < 3; ++i)
	{
		powerEF[i] = 0;
		freq[i] = 0;
	}

	input_dsp_f32_t leftDelta = 0;
	input_dsp_f32_t rightDelta = 0;
	for (uint16_t i = 0; i < dataSize; ++i)
	{
		leftDelta = rightDelta;
		rightDelta = (i == dataSize - 1 ? 0 : inputData[i + 1] - inputData[i]);
		if (leftDelta > 0 && rightDelta <= 0)
		{
			// there is an extremmum
			if (inputData[i] > inputData[freq[0]])
			{
				freq[2] = freq[1];
				freq[1] = freq[0];
				freq[0] = i;
			}
			else if (inputData[i] > inputData[freq[1]])
			{
				freq[2] = freq[1];
				freq[1] = i;
			}
			else if (inputData[i] > inputData[freq[2]])
			{
				freq[2] = i;
			}
		}
	}
	powerEF[0] = inputData[freq[0]];
	powerEF[1] = inputData[freq[1]];
	powerEF[2] = inputData[freq[2]];
}

void SaPrecalcStatDataF32(
	const uint16_t* p_ExtractedFeatures,
	uint16_t p_ExtractedFeaturesCount, 
	uint16_t p_ColIdx,
	const input_dsp_f32_t* p_Data,
	size_t p_Size)
{
	DataBufferF32.Data = p_Data;
	DataBufferF32.Size = p_Size;

	for (int i = 0; i < 3; i++)
	{
		DataBufferF32.Moments[i] = 0;
	}

	const size_t dataSize = p_Size;
	calc_bit_mask_t calcBitMask = 0;
	int8_t moment = 0;	

	PopulateFeatureMask(p_ExtractedFeatures, p_ExtractedFeaturesCount, p_ColIdx, &calcBitMask);

	if (BITMASK_MEAN & calcBitMask)
	{
		float sum = 0;
		
		for (size_t i = 0; i < dataSize; ++i)
		{
			sum += DataBufferF32.Data[i];
		}
		
		const float mean = sum / dataSize;

		DataBufferF32.Mean = mean;
	}

	if (BITMASK_MOMENT_4 & calcBitMask)
	{
		moment = 3;
	}
	else if (BITMASK_MOMENT_3 & calcBitMask)
	{
		moment = 2;
	}
	else if (BITMASK_MOMENT_2 & calcBitMask)
	{
		moment = 1;
	}

	if (moment > 0)
	{
		for (size_t i = 0; i < dataSize; ++i)
		{
			const float zeroMomentValue = (DataBufferF32.Data[i] - DataBufferF32.Mean);

			float momentValue = zeroMomentValue;

			for (int32_t momentIdx = 0; momentIdx < moment; momentIdx++)
			{
				momentValue = momentValue * zeroMomentValue;

				DataBufferF32.Moments[momentIdx] += momentValue;
			}
		}

		for (int32_t momentIdx = 0; momentIdx < moment; momentIdx++)
		{
			DataBufferF32.Moments[momentIdx] = DataBufferF32.Moments[momentIdx] / dataSize;
		}
	}

	if (BITMASK_FFT_ANY & calcBitMask)
	{
		const neuton_dsp_fht_instance* fhtInstance;		

		GetFhtInstanceFromAxisCache(p_ColIdx, &fhtInstance);

		input_dsp_f32_t* windowArray = fhtInstance->windowArray;

		const size_t bufferSize = (fhtInstance->fhtWindowSize < DataBufferF32.Size) ?
			fhtInstance->fhtWindowSize : DataBufferF32.Size;
		
		// Zero fftWindow buffer
		memset(windowArray, 0, fhtInstance->fhtWindowSize * sizeof(input_dsp_f32_t));

		// Copy src data to fftBuffer. Take the smallest size		
		memcpy(windowArray, DataBufferF32.Data, bufferSize * sizeof(input_dsp_f32_t));

		// Minus mean from signal to remove const part of a signal		
		for (size_t i = 0; i < bufferSize; ++i)
			windowArray[i] -= DataBufferF32.Mean;

		// Count FFT
		NeutonDspFhtMakeF32(fhtInstance);

		// Find peaks (only first half of the buffer must be searched)
		CalcSpectrumPeaksF32(
			windowArray,
			fhtInstance->fhtWindowSize / 2, 
			DataBufferF32.FhtPowerArray, 
			DataBufferF32.FhtFreqArray);
	}
	
	if (BITMASK_ZERO_CROSSING & calcBitMask)
	{
		const float mean = DataBufferF32.Mean;
		DataBufferF32.NegMeanCrossing = 0;
		DataBufferF32.PosMeanCrossing = 0;
		float value = DataBufferF32.Data[0] - mean;		
		
		for (size_t i = 1; i < dataSize; ++i)
		{
			const float nextValue = DataBufferF32.Data[i] - mean;

			if (value > 0.f && nextValue < 0.000001f)
				DataBufferF32.NegMeanCrossing++;
			else
			{
				if (value<0.f && nextValue>-0.000001f)
					DataBufferF32.PosMeanCrossing++;
			}
			value = nextValue;
		}
		
		DataBufferF32.MeanCrossing = DataBufferF32.NegMeanCrossing + DataBufferF32.PosMeanCrossing;
	}

	if (BITMASK_MIN & calcBitMask)
	{
		float minValue = DataBufferF32.Data[0];
		
		for (size_t i = 1; i < DataBufferF32.Size; ++i)
		{
			const float value = DataBufferF32.Data[i];
			if (minValue > value)
				minValue = value;
		}
		
		DataBufferF32.Min = minValue;
	}

	if (BITMASK_MAX & calcBitMask)
	{
		float maxValue = DataBufferF32.Data[0];
		
		for (size_t i = 1; i < DataBufferF32.Size; ++i)
		{
			const float value = DataBufferF32.Data[i];
			if (maxValue < value)
				maxValue = value;
		}
		
		DataBufferF32.Max = maxValue;
	}

	if (BITMASK_MOOVING_AVERAGE & calcBitMask)
	{
		const int32_t* inputParamPtr = GetFeatureInputParams(p_ColIdx, EF_PARAM_AMP_FREQUENCY_P2P);
		
		const int32_t windowSize = (*inputParamPtr) > dataSize ? dataSize : *inputParamPtr;
		const int32_t halfWs = windowSize >> 1;
		float maLSum = 0;
		
		for (int32_t i = 0; i < windowSize; i++)
		{
			maLSum += DataBufferF32.Data[i];
		}

		float lfMin = maLSum;
		float lfMax = maLSum;
		float hfMin = DataBufferF32.Data[halfWs] * windowSize - maLSum;
		float hfMax = hfMin;
		const int32_t maSize = dataSize - windowSize;
		
		for (int32_t i = 0; i < maSize; i++)
		{
			maLSum = maLSum - DataBufferF32.Data[i] + DataBufferF32.Data[i + windowSize];
			
			if (BITMASK_AMP_LOW_FREQUENCY_P2P & calcBitMask)
			{
				if (lfMin > maLSum)
					lfMin = maLSum;
				if (lfMax < maLSum)
					lfMax = maLSum;
			}
			
			if (BITMASK_AMP_HIGH_FREQUENCY_P2P & calcBitMask)
			{
				const float maHSum = DataBufferF32.Data[i + halfWs + 1] * windowSize - maLSum;
				if (hfMin > maHSum)
					hfMin = maHSum;
				if (hfMax < maHSum)
					hfMax = maHSum;
			}
		}
		
		DataBufferF32.AmpP2PLF = (lfMax - lfMin) / windowSize;
		DataBufferF32.AmpP2PHF = (hfMax - hfMin) / windowSize;
	}
}

output_dsp_f32_t MinF32()
{
	return DataBufferF32.Min;
}

output_dsp_f32_t MaxF32()
{
	return DataBufferF32.Max;
}

output_dsp_f32_t MeanF32()
{
	return DataBufferF32.Mean;
}

output_dsp_f32_t RootMeanSquareF32()
{
	return sqrtf(DataBufferF32.Moments[MOMENT_2]);
}

output_dsp_f32_t MeanCrossingF32()
{
	return DataBufferF32.MeanCrossing;
}

output_dsp_f32_t VarianceF32()
{
	return DataBufferF32.Moments[MOMENT_2];
}

output_dsp_f32_t PetrosianFractalDimensionF32()
{
	const float lg = log10f(DataBufferF32.Size);
	return lg / (lg + log10f(DataBufferF32.Size / (DataBufferF32.Size + 0.4*DataBufferF32.MeanCrossing)));
}

output_dsp_f32_t SkewnessF32()
{
	if (DataBufferF32.Moments[MOMENT_2] < 1E-6)
		return 0.f;

	return DataBufferF32.Moments[MOMENT_3] / sqrtf(DataBufferF32.Moments[MOMENT_2] * DataBufferF32.Moments[MOMENT_2] * DataBufferF32.Moments[MOMENT_2]);
}

output_dsp_f32_t KurtosisF32()
{
	if (DataBufferF32.Moments[MOMENT_2] < 1E-6)
		return 0.f;

	return (DataBufferF32.Moments[MOMENT_4] / (DataBufferF32.Moments[MOMENT_2] * DataBufferF32.Moments[MOMENT_2])) - 3;
}

output_dsp_f32_t NegMeanCrossingF32()
{
	return DataBufferF32.NegMeanCrossing;
}

output_dsp_f32_t PosMeanCrossingF32()
{
	return DataBufferF32.PosMeanCrossing;
}

output_dsp_f32_t AmplitudeGlobalP2pHighFrequencyF32()
{
	return DataBufferF32.AmpP2PHF;
}

output_dsp_f32_t AmplitudeGlobalP2pLowFrequencyF32()
{
	return DataBufferF32.AmpP2PLF;
}

output_dsp_f32_t FhtPower0F32()
{
	return DataBufferF32.FhtPowerArray[0];
}

output_dsp_f32_t FhtPower1F32()
{
	return DataBufferF32.FhtPowerArray[1];
}

output_dsp_f32_t FhtPower2F32()
{
	return DataBufferF32.FhtPowerArray[2];
}

output_dsp_f32_t FhtFreq0F32()
{
	return DataBufferF32.FhtFreqArray[0];
}

output_dsp_f32_t FhtFreq1F32()
{
	return DataBufferF32.FhtFreqArray[1];
}

output_dsp_f32_t FhtFreq2F32()
{
	return DataBufferF32.FhtFreqArray[2];
}
