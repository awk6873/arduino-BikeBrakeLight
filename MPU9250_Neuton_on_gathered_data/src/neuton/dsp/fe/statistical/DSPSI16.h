/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_FE_STATISTICAL_DSPSI16_H
#define NEUTON_FE_STATISTICAL_DSPSI16_H

#include "Common.h"

typedef int16_t input_dsp_si16_t;
typedef int32_t output_dsp_si16_t;

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
void SaPrecalcStatDataSI16(
	const uint16_t* p_ExtractedFeatures,
	uint16_t p_ExtractedFeaturesCount,
	uint16_t p_ColIdx,
	const input_dsp_si16_t* p_Data,
	size_t p_Size);
	
/**
 * \brief Compute the minimum values simultaneously.
 */
output_dsp_si16_t MinSI16();

/**
 * \brief Compute the maximum values simultaneously.
 */
output_dsp_si16_t MaxSI16();

/**
 * \brief Compute the arithmetic mean.
 */
output_dsp_si16_t MeanSI16();

/**
 * \brief Compute the root mean square, [RMS](https://en.wikipedia.org/wiki/Root_mean_square).
 */
output_dsp_si16_t RootMeanSquareSI16();

/**
 * \brief Count the number of sign changes
 */
output_dsp_si16_t MeanCrossingSI16();

/**
 * \brief Count the number of mean crossing in down direction
 */
output_dsp_si16_t NegMeanCrossingSI16();

/**
 * \brief Count the number of mean crossing in up direction
 */
output_dsp_si16_t PosMeanCrossingSI16();

/**
 * \brief Compute the variance
 */
output_dsp_si16_t VarianceSI16();

/**
 * \brief Compute the Petrosian fractal dimension, PFD.
 */
output_dsp_si16_t PetrosianFractalDimensionSI16();

/**
 * \brief Compute the [skewness](https://en.wikipedia.org/wiki/Skewness)
 */
output_dsp_si16_t SkewnessSI16();

/**
 * \brief  Compute the [kurtosis](https://en.wikipedia.org/wiki/Kurtosis)
 */
output_dsp_si16_t KurtosisSI16();

/**
 * \brief Calculates the peak-to-peak (min-max) of the curve after subtracting the low-frequency component (smoothed curve with a nsmooth parameter)
 */
output_dsp_si16_t AmplitudeGlobalP2pHighFrequencySI16();

/**
 * \brief Calculates the peak-to-peak (min-max) of the smoothed curve with a nsmooth parameter
 */
output_dsp_si16_t AmplitudeGlobalP2pLowFrequencySI16();

/**
* \brief Calculates Fht pow 1
*/
output_dsp_si16_t FhtPower0SI16();

/**
* \brief Calculates Fht pow 2
*/
output_dsp_si16_t FhtPower1SI16();

/**
* \brief Calculates Fht pow 3
*/
output_dsp_si16_t FhtPower2SI16();

/**
* \brief Calculates Fht freq 1
*/
output_dsp_si16_t FhtFreq0SI16();

/**
* \brief Calculates Fht freq 2
*/
output_dsp_si16_t FhtFreq1SI16();

/**
* \brief Calculates Fht freq 3
*/
output_dsp_si16_t FhtFreq2SI16();

#ifdef __cplusplus
}
#endif

#endif // NEUTON_FE_STATISTICAL_DSPSI16_H
