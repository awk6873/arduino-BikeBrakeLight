/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_FE_STATISTICAL_DSPF32_H
#define NEUTON_FE_STATISTICAL_DSPF32_H

#include "Common.h"

typedef float input_dsp_f32_t;
typedef float output_dsp_f32_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Init common data buffer to optimize calculations
 * \param p_ExtractedFeatures Array of ExtractedFeatures enum values. (extracted features for current axis)
 * \param p_ExtractedFeaturesCount Number of elements in p_ExtractedFeatures array
 * \param p_ColIdx Current axis index. Used to get input parameters for some features
 * \param p_Data Pointer to processed memory buffer
 * \param p_Size Size of the buffer (n elements)
 */
void SaPrecalcStatDataF32(
	const uint16_t* p_ExtractedFeatures,
	uint16_t p_ExtractedFeaturesCount, 
	uint16_t p_ColIdx,
	const input_dsp_f32_t* p_Data,
	size_t p_Size);

/**
 * \brief Compute the minimum values simultaneously.
 */
output_dsp_f32_t MinF32();

/**
 * \brief Compute the maximum values simultaneously.
 */
output_dsp_f32_t MaxF32();

/**
 * \brief Compute the arithmetic mean.
 */
output_dsp_f32_t MeanF32();

/**
 * \brief Compute the root mean square, [RMS](https://en.wikipedia.org/wiki/Root_mean_square).
 */
output_dsp_f32_t RootMeanSquareF32();

/**
 * \brief Count the number of sign changes
 */
output_dsp_f32_t MeanCrossingF32();

/**
 * \brief Count the number of mean crossing in down direction
 */
output_dsp_f32_t NegMeanCrossingF32();

/**
 * \brief Count the number of mean crossing in up direction
 */
output_dsp_f32_t PosMeanCrossingF32();

/**
 * \brief Compute the variance
 */
output_dsp_f32_t VarianceF32();

/**
 * \brief Compute the Petrosian fractal dimension, PFD.
 */
output_dsp_f32_t PetrosianFractalDimensionF32();

/**
 * \brief Compute the [skewness](https://en.wikipedia.org/wiki/Skewness)
 */
output_dsp_f32_t SkewnessF32();

/**
 * \brief  Compute the [kurtosis](https://en.wikipedia.org/wiki/Kurtosis)
 */
output_dsp_f32_t KurtosisF32();

/**
 * \brief Calculates the peak-to-peak (min-max) of the curve after subtracting the low-frequency component (smoothed curve with a nsmooth parameter)
 */
output_dsp_f32_t AmplitudeGlobalP2pHighFrequencyF32();

/**
 * \brief Calculates the peak-to-peak (min-max) of the smoothed curve with a nsmooth parameter
 */
output_dsp_f32_t AmplitudeGlobalP2pLowFrequencyF32();

/**
 * \brief Calculates Fht pow 1
 */
output_dsp_f32_t FhtPower0F32();

/**
* \brief Calculates Fht pow 2
*/
output_dsp_f32_t FhtPower1F32();

/**
* \brief Calculates Fht pow 3
*/
output_dsp_f32_t FhtPower2F32();

/**
* \brief Calculates Fht freq 1
*/
output_dsp_f32_t FhtFreq0F32();

/**
* \brief Calculates Fht freq 2
*/
output_dsp_f32_t FhtFreq1F32();

/**
* \brief Calculates Fht freq 3
*/
output_dsp_f32_t FhtFreq2F32();
	
#ifdef __cplusplus
}
#endif

#endif // NEUTON_FE_STATISTICAL_DSPF32_H
