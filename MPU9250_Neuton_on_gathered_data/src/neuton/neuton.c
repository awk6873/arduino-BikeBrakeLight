/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "neuton.h"
#include <stdlib.h>

#if (NEUTON_MODEL_QLEVEL == 32)
	#include "preprocessing/blocks/normalize/normalize.h"
#endif

#if (NEUTON_MODEL_FLOAT_SUPPORT == 1)
	#include <math.h>
#endif

// '__attribute__ ((aligned))' is used only in gcc compiler for MC purposes 
#ifdef _MSC_VER
#define ATTRIBUTE_ALIGNED_ARRAY
#else
#define ATTRIBUTE_ALIGNED_ARRAY __attribute__ ((aligned))
#endif


#define N_ELEMENTS(arr) (sizeof(arr) / sizeof(arr[0]))
#define MAX_INPUT_FLOAT	0.9999999f

#if (NEUTON_MODEL_HEADER_VERSION < 3)
#define NEUTON_INPUTS_IS_INTEGER 0
#endif


static float modelOutput[NEUTON_MODEL_OUTPUTS_COUNT];
static coeff_t modelAccumulators[NEUTON_MODEL_NEURONS_COUNT];
static uint8_t modelIsReadyForInference = 0;

#if (NEUTON_MODEL_QLEVEL < 32)
static const coeff_t ctMax = ((uint32_t)(1) << NEUTON_MODEL_QLEVEL) - 1;
#endif

#define NEUTON_MODEL_INPUT_TYPE_SIZE ((sizeof(input_t) > sizeof(coeff_t)) ? sizeof(input_t) : sizeof(coeff_t))

#if (NEUTON_PREPROCESSING_ENABLED == 0)

	static uint8_t modelInputBuffer[NEUTON_MODEL_INPUTS_COUNT * NEUTON_MODEL_INPUT_TYPE_SIZE] ATTRIBUTE_ALIGNED_ARRAY;

#else

	#include "dsp/fe/statistical/DSP.h"
	#include "model/config.h"

	static uint8_t modelInputBuffer[NEUTON_MODEL_USED_ORIGINAL_INPUTS_COUNT * NEUTON_MODEL_WINDOW_SIZE * NEUTON_MODEL_INPUT_TYPE_SIZE] ATTRIBUTE_ALIGNED_ARRAY;
	static uint16_t modelWindowFill = 0;

#if (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)
	#define NEUTON_MODEL_EXTRACTED_FEATURES_TYPE_SIZE ((sizeof(extracted_feature_t) > sizeof(coeff_t)) ? sizeof(extracted_feature_t) : sizeof(coeff_t))
	static uint8_t extractedFeaturesBuffer[NEUTON_MODEL_EXTRACTED_FEATURES_COUNT * NEUTON_MODEL_EXTRACTED_FEATURES_TYPE_SIZE] ATTRIBUTE_ALIGNED_ARRAY;
#endif

#endif

#if NEUTON_MODEL_QLEVEL==8
	typedef uint16_t double_qu_t;
	typedef int16_t double_qs_t;
#elif NEUTON_MODEL_QLEVEL==16
	typedef uint32_t double_qu_t;
	typedef int32_t double_qs_t;
#else
	typedef float double_qu_t;
	typedef float double_qs_t;
#endif

extern inline uint8_t neuton_model_quantization_level()
{
	return NEUTON_MODEL_QLEVEL;
}

extern inline uint8_t neuton_model_float_calculations()
{
	return NEUTON_MODEL_FLOAT_SUPPORT;
}

extern inline TaskType neuton_model_task_type()
{
	return (TaskType) NEUTON_MODEL_TASK_TYPE;
}

extern inline uint16_t neuton_model_outputs_count()
{
	return NEUTON_MODEL_OUTPUTS_COUNT;
}

extern inline uint16_t neuton_model_neurons_count()
{
	return NEUTON_MODEL_NEURONS_COUNT;
}

extern inline uint32_t neuton_model_weights_count()
{
	return NEUTON_MODEL_WEIGHTS_COUNT;
}

extern inline uint16_t neuton_model_inputs_limits_count()
{
	return NEUTON_MODEL_INPUT_LIMITS_COUNT;
}

extern inline uint16_t neuton_model_inputs_count()
{
#if (NEUTON_PREPROCESSING_ENABLED == 0)

	return NEUTON_MODEL_INPUTS_COUNT;

#else

	return NEUTON_MODEL_INPUTS_COUNT_ORIGINAL;

#endif
}

extern inline uint16_t neuton_model_window_size()
{
#if (NEUTON_PREPROCESSING_ENABLED == 1)

	return NEUTON_MODEL_WINDOW_SIZE;

#else

	return 1;

#endif
}

uint32_t neuton_model_ram_usage()
{
	return sizeof(modelOutput) + sizeof(modelAccumulators) + sizeof(modelIsReadyForInference)
			+ sizeof(modelInputBuffer)

#if (NEUTON_PREPROCESSING_ENABLED == 1)
			+ sizeof(modelWindowFill)

#endif
#if (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)
			+ sizeof(extractedFeaturesBuffer)
#endif
#if (NEUTON_MODEL_QLEVEL < 32)
			+ sizeof(ctMax)
#endif
#if (CONTAINS_FFT_FEATURES > 0)
		+ sizeof(fftWindow)
#endif
			;
}

uint32_t neuton_model_size()
{
	return sizeof(modelWeights)	+ sizeof(modelLinks) + sizeof(modelFuncCoeffs)
			+ sizeof(modelIntLinksBoundaries) + sizeof(modelExtLinksBoundaries)
			+ sizeof(modelOutputNeurons)

#if (NEUTON_MODEL_HEADER_VERSION > 1)

			+ sizeof(modelFuncTypes)

#endif
			;
}

uint32_t neuton_model_size_with_meta()
{
	return neuton_model_size()

#if (NEUTON_PREPROCESSING_ENABLED == 0)

			+ sizeof(modelInputMin)	+ sizeof(modelInputMax)

#endif
#if (NEUTON_PREPROCESSING_ENABLED == 1)

	#if (NEUTON_DROP_ORIGINAL_FEATURES != 1)
			+ sizeof(modelInputScaleMin) + sizeof(modelInputScaleMax)
	#endif

	#if (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)
			+ sizeof(extractedFeaturesScaleMin) + sizeof(extractedFeaturesScaleMax)
	#endif

#endif
#if (NEUTON_BITMASK_ENABLED == 1)

			+ sizeof(modelUsedInputsMask)

#endif
#if (NEUTON_MODEL_TASK_TYPE == 2)

			+ sizeof(modelOutputMin) + sizeof(modelOutputMax)

#endif
#if (NEUTON_MODEL_LOG_SCALE_OUTPUTS == 1)

			+ sizeof(modelOutputLogFlag) + sizeof(modelOutputLogScale)

#endif
#if (NEUTON_PREPROCESSING_ENABLED == 1)

			+ sizeof(modelOriginalFeatureUsed)

		#if (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)

			+ sizeof(modelExtractedFeaturesStartIdxForAxle)
			+ sizeof(modelExtractedFeaturesCountForAxle)
			+ sizeof(modelExtractedFeaturesParamsOffset)
			+ sizeof(modelExtractedFeatures)
			+ sizeof(modelExtractedFeaturesParams)

		#endif

#endif

#if (CONTAINS_FFT_FEATURES > 0)

		+ sizeof(fftWindowSize)
		+ sizeof(fftLogN)
		+ sizeof(fftReversedBitIndexArray)
		+ sizeof(fftSinTable)
		+ sizeof(fftTgTable)
		+ sizeof(fftWindow)

#endif

#if (AUDIO_KWS_ENABLED > 0)
	
		+ sizeof(kwsSamplingRate)
		+ sizeof(kwsAudioDuration)
		+ sizeof(kwsWindowLen)
		+ sizeof(kwsMelFreqBands)
		+ sizeof(kwsMelTimeBands)
		+ sizeof(kwsWindowHop)
	
#endif
			;
}

static void denormalize_outputs()
{
#if (NEUTON_MODEL_TASK_TYPE == 0) || (NEUTON_MODEL_TASK_TYPE == 1)

	float sum = 0;

	for (uint16_t i = 0; i < NEUTON_MODEL_OUTPUTS_COUNT; ++i)
		sum += modelOutput[i];

	for (uint16_t i = 0; i < NEUTON_MODEL_OUTPUTS_COUNT; ++i)
		modelOutput[i] = (sum != 0) ? modelOutput[i] / sum: 0;

#endif

#if (NEUTON_MODEL_TASK_TYPE == 2)

	for (uint16_t i = 0; i < NEUTON_MODEL_OUTPUTS_COUNT; ++i)
	{
		modelOutput[i] = modelOutput[i] * (modelOutputMax[i] - modelOutputMin[i]) + modelOutputMin[i];

#if (NEUTON_MODEL_LOG_SCALE_OUTPUTS == 1)

		if (modelOutputLogFlag[i])
			modelOutput[i] = expf(modelOutput[i]) - modelOutputLogScale[i];

#endif

	}
#endif
}

void neuton_model_reset_inputs()
{
#if (NEUTON_PREPROCESSING_ENABLED != 0)

	modelWindowFill = 0;

#endif

	modelIsReadyForInference = 0;
}

int8_t neuton_model_set_inputs(input_t *inputs)
{
	if (!inputs)
		return -1;

	input_t* buffer = (input_t*)modelInputBuffer;

#if (NEUTON_PREPROCESSING_ENABLED == 0)

	for (uint16_t i = 0; i < neuton_model_inputs_count(); ++i)
		buffer[i] = inputs[i];

	modelIsReadyForInference = 1;

	return 0;

#else

	uint16_t column = 0;
	for (uint16_t i = 0; i < neuton_model_inputs_count(); ++i)
	{
	#if (NEUTON_MODEL_USED_ORIGINAL_INPUTS_COUNT != NEUTON_MODEL_INPUTS_COUNT_ORIGINAL)
		if (modelOriginalFeatureUsed[i >> 3] & ((uint8_t)(1) << (i % 8)))
	#endif
		{
			buffer[column * NEUTON_MODEL_WINDOW_SIZE + modelWindowFill] = inputs[i];
			++column;
		}
	}

	if (++modelWindowFill >= NEUTON_MODEL_WINDOW_SIZE)
	{
		modelWindowFill = 0;
		modelIsReadyForInference = 1;

		return 0;
	}

	return 1;

#endif
}

void neuton_model_set_ready_flag()
{

#if (NEUTON_PREPROCESSING_ENABLED != 0)
	modelWindowFill = 0;
#endif

	modelIsReadyForInference = 1;
}

input_t* neuton_model_get_inputs_ptr()
{
	return (input_t*) modelInputBuffer;
}

#if (NEUTON_MODEL_QLEVEL == 32)

static inline coeff_t neuton_activation_fn(neurons_size_t neuronIndex, acc_signed_t summ)
{
	return 1.0f / (1.0f + expf((acc_signed_t) ((acc_signed_t) -modelFuncCoeffs[neuronIndex]) * summ));
}

#else // (NEUTON_MODEL_QLEVEL == 32)

#if (NEUTON_MODEL_FLOAT_SUPPORT == 0)

	static coeff_t accurate_fast_sigmoid(acc_signed_t arg)
	{
		coeff_t qResult = 0;
		coeff_t secondPointY = 0;
		coeff_t firstPointY = 0;

		static const uint8_t QLVL = NEUTON_MODEL_QLEVEL;
		static const uint8_t QLVLM1 = NEUTON_MODEL_QLEVEL - 1;

		const coeff_t intPart = abs(arg) / ((uint32_t)(1) << QLVL);
		const coeff_t realPart = abs(arg) - (intPart << QLVL);

		if (intPart == 0 && realPart == 0)
		{
			return (uint32_t)(1) << QLVLM1;
		}

		uint8_t s = arg < 0;
		uint8_t odd = 1;

		if (realPart == 0)
		{
			for (uint8_t i = 0; i < QLVL; i++)
			{
				const uint8_t bit = ((i / intPart + s) & odd);
				qResult = qResult | (bit << (QLVLM1 - i));
			}
			return qResult;
		}

		const coeff_t secondPointX = intPart + 1;
		if (intPart == 0)
		{
			firstPointY = (uint32_t)(1) << QLVLM1;
			for (uint8_t i = 0; i < QLVL; i++)
			{
				const uint8_t bit = ((i / secondPointX) & odd);
				secondPointY = secondPointY | (bit << (QLVLM1 - i));
			}
		}
		else
		{
			if (secondPointX == 0)
			{
				for (uint8_t i = 0; i < QLVL; i++)
				{
					const uint8_t bit = ((i / intPart) & odd);
					firstPointY = firstPointY | (bit << (QLVLM1 - i));
				}
				secondPointY = (uint32_t)(1) << QLVLM1;
			}
			else
			{
				for (uint8_t i = 0; i < QLVL; i++)
				{
					uint8_t bit = ((i / intPart) & odd);
					firstPointY = firstPointY | (bit << (QLVLM1 - i));
					bit = ((i / secondPointX) & odd);
					secondPointY = secondPointY | (bit << (QLVLM1 - i));
				}
			}
		}

		const coeff_t res = firstPointY + ((realPart * (secondPointY - firstPointY)) >> QLVL);
		if (s)
			return res == 0 ? ctMax : ctMax + 1 - res;

		return res;
	}

#endif // (NEUTON_MODEL_FLOAT_SUPPORT == 0)

static inline float neuton_deqantize_value(coeff_t value)
{
	return (float) value / (float) ((uint32_t)(1) << NEUTON_MODEL_QLEVEL);
}

#if (NEUTON_MODEL_QLEVEL == 8)
	#define KSHIFT 2
#endif

#if (NEUTON_MODEL_QLEVEL == 16)
	#define KSHIFT 10
#endif

static coeff_t neuton_activation_fn(neurons_size_t neuronIndex, acc_signed_t summ)
{

#if (NEUTON_MODEL_FLOAT_SUPPORT == 1)

	const float qs = (float) (((acc_signed_t) modelFuncCoeffs[neuronIndex] * summ)
			>> (NEUTON_MODEL_QLEVEL + KSHIFT - 1)) / (float) ((uint32_t)(1) << (NEUTON_MODEL_QLEVEL));

	const float tmpValue = 1.0f / (1.0f + expf(-qs));

	return (tmpValue > MAX_INPUT_FLOAT ? MAX_INPUT_FLOAT : tmpValue) * (float) ((uint32_t)(1) << NEUTON_MODEL_QLEVEL);

#else // (NEUTON_MODEL_FLOAT_SUPPORT == 1)

	return accurate_fast_sigmoid(
		-(((acc_signed_t) modelFuncCoeffs[neuronIndex] * summ) >> (NEUTON_MODEL_QLEVEL + KSHIFT - 1))
	);

#endif // (NEUTON_MODEL_FLOAT_SUPPORT == 1)

}
#endif // (NEUTON_MODEL_QLEVEL == 32)


#if (NEUTON_BITMASK_ENABLED == 1)
static inline uint8_t is_input_used(uint32_t pos)
{
	return (modelUsedInputsMask[pos >> 3] & ((uint8_t)(1) << (pos % 8)));
}
#endif

#if (NEUTON_DROP_ORIGINAL_FEATURES != 1)

static coeff_t prepare_model_input(input_t value, uint16_t index)
{
	input_t min, max;

	(void)index;

#if (NEUTON_PREPROCESSING_ENABLED == 1)

	min = modelInputScaleMin[index / NEUTON_MODEL_WINDOW_SIZE];
	max = modelInputScaleMax[index / NEUTON_MODEL_WINDOW_SIZE];

#else

	#if (NEUTON_MODEL_INPUT_LIMITS_COUNT == 1)

		min = modelInputMin[0];
		max = modelInputMax[0];

	#else

		min = modelInputMin[index];
		max = modelInputMax[index];

	#endif

#endif

#if (NEUTON_MODEL_QLEVEL == 32)

	coeff_t ct = value;
	NeutonPreprocessingBlockNormalize(&ct, &ct, 1, min, max);
	return ct;

#else

	if (value < min)
		value = min;
	if (value > max)
		value = max;

	#if (NEUTON_INPUTS_IS_INTEGER == 1)

		uint64_t tmp = value - min;

		if ((max - min) == 0)
		{
			return tmp * ctMax;
		}
		else
		{
			return tmp * ctMax / (max - min);
		}

	#else

		if ((max - min) == 0)
		{
			return (value - min) * ctMax;
		}
		else
		{
			return (value - min) * ctMax / (max - min);
		}

	#endif

#endif
}

static void prepare_model_inputs()
{
	input_t* src = (input_t*)modelInputBuffer;
	coeff_t* dst = (coeff_t*)modelInputBuffer;

	static const uint16_t count = sizeof(modelInputBuffer) / NEUTON_MODEL_INPUT_TYPE_SIZE;

	if (sizeof(coeff_t) > sizeof(input_t))
	{
		for (uint16_t i = count-1; ; --i)
		{
		#if (NEUTON_BITMASK_ENABLED == 1)
			if (is_input_used(i))
		#endif
				dst[i] = prepare_model_input(src[i], i);

			if (i == 0) break;
		}
	}
	else
	{
		for (uint16_t i = 0; i < count; ++i)
		{
		#if (NEUTON_BITMASK_ENABLED == 1)
			if (is_input_used(i))
		#endif
				dst[i] = prepare_model_input(src[i], i);
		}
	}
}

#endif // (NEUTON_DROP_ORIGINAL_FEATURES != 1)

#if (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)

static void extract_features()
{
	input_t* src = (input_t*)modelInputBuffer;
	extracted_feature_t* dst = (extracted_feature_t*)extractedFeaturesBuffer;

#if (NEUTON_DROP_ORIGINAL_FEATURES == 1)
	uint32_t shift = 0;
#else
	uint32_t shift = sizeof(modelInputBuffer) / NEUTON_MODEL_INPUT_TYPE_SIZE;
#endif

	dsp_init_lib(modelExtractedFeatures);

	for (uint16_t column = 0; column < NEUTON_MODEL_USED_ORIGINAL_INPUTS_COUNT; ++column)
	{
		const uint16_t efCountForCol = modelExtractedFeaturesCountForAxle[column];

		if (efCountForCol > 0)
		{
			const uint32_t efStartIdx = modelExtractedFeaturesStartIdxForAxle[column];

			SaPrecalcStatData(
				modelExtractedFeatures + efStartIdx,
				efCountForCol,
				column,
				src, 
				NEUTON_MODEL_WINDOW_SIZE);

			// enum features for chosen column

			const uint32_t featureEndIdx = efStartIdx + efCountForCol;

			for (uint32_t featureIdx = efStartIdx, feature = 0; featureIdx < featureEndIdx; ++featureIdx)
			{
#if (NEUTON_BITMASK_ENABLED == 1)
				if (is_input_used(shift + feature))
#endif
				switch (modelExtractedFeatures[featureIdx])
				{
				case EF_STAT_MIN:
					dst[feature] = SaMin();
					break;
				case EF_STAT_MAX:
					dst[feature] = SaMax();
					break;
				case EF_STAT_MEAN:
					dst[feature] = SaMean();
					break;
				case EF_STAT_RMS:
					dst[feature] = SaRootMeanSquare();
					break;
				case EF_STAT_MEAN_CROSSING:
					dst[feature] = SaMeanCrossing();
					break;
				case EF_STAT_NEGATIVE_MEAN_CROSSING:
					dst[feature] = SaNegMeanCrossing();
					break;
				case EF_STAT_POSITIVE_MEAN_CROSSING:
					dst[feature] = SaPosMeanCrossing();
					break;
				case EF_STAT_VARIANCE:
					dst[feature] = SaVariance();
					break;
				case EF_STAT_PFD:
					dst[feature] = SaPetrosianFractalDimension();
					break;
				case EF_STAT_SKEWNESS:
					dst[feature] = SaSkewness();
					break;
				case EF_STAT_KURTOSIS:
					dst[feature] = SaKurtosis();
					break;
				case EF_AMP_HIGH_FREQUENCY_P2P:
					dst[feature] = SaAmplitudeGlobalP2pHighFrequency();
					break;
				case EF_AMP_LOW_FREQUENCY_P2P:
					dst[feature] = SaAmplitudeGlobalP2pLowFrequency();
					break;
				case EF_FFT_POWER0:
					dst[feature] = SaFhtPower0();
					break;
				case EF_FFT_POWER1:
					dst[feature] = SaFhtPower1();
					break;
				case EF_FFT_POWER2:
					dst[feature] = SaFhtPower2();
					break;
				case EF_FFT_FREQ0:
					dst[feature] = SaFhtFreq0();
					break;
				case EF_FFT_FREQ1:
					dst[feature] = SaFhtFreq1();
					break;
				case EF_FFT_FREQ2:
					dst[feature] = SaFhtFreq2();
					break;
				case EF_STAT_UNUSED:
				default:
					break;
				}

				++feature;
			}

		} // efCountForCol > 0

		// move buffers to next column
		src += NEUTON_MODEL_WINDOW_SIZE;
		dst += efCountForCol;
		shift += efCountForCol;

	} // for column
}

static coeff_t prepare_extracted_feature(extracted_feature_t value, uint16_t index)
{
	extracted_feature_t min = extractedFeaturesScaleMin[index];
	extracted_feature_t max = extractedFeaturesScaleMax[index];

#if (NEUTON_MODEL_QLEVEL == 32)

	coeff_t ct = value;
	NeutonPreprocessingBlockNormalize(&ct, &ct, 1, min, max);
	return ct;

#else

	if (value < min)
		value = min;
	if (value > max)
		value = max;

	#if (NEUTON_INPUTS_IS_INTEGER == 1)

		uint64_t tmp = value - min;

		if ((max - min) == 0)
		{
			return tmp * ctMax;
		}
		else
		{
			return tmp * ctMax / (max - min);
		}

	#else

		if ((max - min) == 0)
		{
			return (value - min) * ctMax;
		}
		else
		{
			return (value - min) * ctMax / (max - min);
		}

	#endif

#endif
}

static void prepare_extracted_features()
{
	extracted_feature_t* src = (extracted_feature_t*)extractedFeaturesBuffer;
	coeff_t* dst = (coeff_t*)extractedFeaturesBuffer;
	static const uint16_t count = sizeof(extractedFeaturesBuffer) / NEUTON_MODEL_EXTRACTED_FEATURES_TYPE_SIZE;

#if (NEUTON_BITMASK_ENABLED == 1)
	#if (NEUTON_DROP_ORIGINAL_FEATURES == 1)
		uint32_t shift = 0;
	#else
		uint32_t shift = sizeof(modelInputBuffer) / NEUTON_MODEL_INPUT_TYPE_SIZE;
	#endif
#endif

	if (sizeof(coeff_t) > sizeof(input_t))
	{
		for (uint16_t i = count-1; ; --i)
		{
		#if (NEUTON_BITMASK_ENABLED == 1)
			if (is_input_used(i + shift))
		#endif
				dst[i] = prepare_extracted_feature(src[i], i);

			if (i == 0) break;
		}
	}
	else
	{
		for (uint16_t i = 0; i < count; ++i)
		{
		#if (NEUTON_BITMASK_ENABLED == 1)
			if (is_input_used(i + shift))
		#endif
				dst[i] = prepare_extracted_feature(src[i], i);
		}
	}
}

#endif // (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)

static inline coeff_t get_model_input(uint16_t index)
{
#if (NEUTON_PREPROCESSING_ENABLED == 0)
	static const uint16_t inputsCount = NEUTON_MODEL_INPUTS_COUNT;
#else
	static const uint16_t inputsCount = sizeof(modelInputBuffer) / NEUTON_MODEL_INPUT_TYPE_SIZE;
#endif

	coeff_t* coeff;

#if (NEUTON_DROP_ORIGINAL_FEATURES == 1)
	index += inputsCount;
#else
	if (index < inputsCount)
	{
		coeff = (coeff_t*)modelInputBuffer;
		return coeff[index];
	}
#endif

#if (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)
	index -= inputsCount;
	if (index < NEUTON_MODEL_EXTRACTED_FEATURES_COUNT)
	{
		coeff = (coeff_t*)extractedFeaturesBuffer;
		return coeff[index];
	}
#endif

#if (NEUTON_MODEL_QLEVEL == 32)
	return 1.0f;
#else
	return ctMax;
#endif
}


int8_t neuton_model_run_inference(uint16_t *index, float **outputs)
{
	if (!modelIsReadyForInference)
		return 1;

#if (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)

	extract_features();
	prepare_extracted_features();

#endif

#if (NEUTON_DROP_ORIGINAL_FEATURES != 1)
	prepare_model_inputs();
#endif

	weights_size_t weightIndex = 0;
	for (neurons_size_t neuronIndex = 0; neuronIndex < NEUTON_MODEL_NEURONS_COUNT; ++neuronIndex)
	{
		acc_signed_t sum = 0;
		double_qs_t mul;

		weights_size_t boundary = modelIntLinksBoundaries[neuronIndex];
		while (weightIndex < boundary)
		{
			const double_qs_t firstValue  = modelWeights[weightIndex];
			const double_qs_t secondValue = modelAccumulators[modelLinks[weightIndex]];
			mul = firstValue * secondValue;
			sum += mul;
			++weightIndex;
		}

		boundary = modelExtLinksBoundaries[neuronIndex];
		while (weightIndex < boundary)
		{
			const double_qs_t firstValue  = modelWeights[weightIndex];
			const double_qs_t secondValue = get_model_input(modelLinks[weightIndex]);
			mul = firstValue * secondValue;
			sum += mul;
			++weightIndex;
		}

		modelAccumulators[neuronIndex] = neuton_activation_fn(neuronIndex, sum);
	}

	for (neurons_size_t i = 0; i < NEUTON_MODEL_OUTPUTS_COUNT; ++i)
	{
#if (NEUTON_MODEL_QLEVEL == 32)

		modelOutput[i] = modelAccumulators[modelOutputNeurons[i]];

#else

		modelOutput[i] = neuton_deqantize_value(modelAccumulators[modelOutputNeurons[i]]);

#endif
	}


	denormalize_outputs();


#if (NEUTON_MODEL_OUTPUTS_COUNT == 1)

	if (index)
		*index = 0;

#else

	if (index)
	{
		uint16_t target = 0;
		float max = 0.0;

		for (uint16_t i = 0; i < NEUTON_MODEL_OUTPUTS_COUNT; ++i)
			if (max < modelOutput[i])
			{
				max = modelOutput[i];
				target = i;
			}

		*index = target;
	}

#endif

	if (outputs)
		*outputs = modelOutput;

	return 0;
}
