/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_FE_STATISTICAL_DSP_H
#define NEUTON_FE_STATISTICAL_DSP_H

#include "Common.h"

#if (NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)

#if !defined(SA_PRECISION)

	#error SA_PRECISION should be defined

#elif SA_PRECISION == 24

	#include "DSPF32.h"

	#define SaInitLib InitLibF32
	#define SaPrecalcStatData SaPrecalcStatDataF32
	#define SaMin MinF32
	#define SaMax MaxF32
	#define SaMean MeanF32
	#define SaRootMeanSquare RootMeanSquareF32
	#define SaMeanCrossing MeanCrossingF32
	#define SaNegMeanCrossing NegMeanCrossingF32
	#define SaPosMeanCrossing PosMeanCrossingF32
	#define SaVariance VarianceF32
	#define SaPetrosianFractalDimension PetrosianFractalDimensionF32
	#define SaSkewness SkewnessF32
	#define SaKurtosis KurtosisF32
	#define SaAmplitudeGlobalP2pLowFrequency AmplitudeGlobalP2pLowFrequencyF32
	#define SaAmplitudeGlobalP2pHighFrequency AmplitudeGlobalP2pHighFrequencyF32
	#define SaFhtPower0 FhtPower0F32
	#define SaFhtPower1 FhtPower1F32
	#define SaFhtPower2 FhtPower2F32
	#define SaFhtFreq0 FhtFreq0F32
	#define SaFhtFreq1 FhtFreq1F32
	#define SaFhtFreq2 FhtFreq2F32

#elif SA_PRECISION == 7

	#include "DSPSI8.h"

	#define SaInitLib InitLibSI8
	#define SaPrecalcStatData SaPrecalcStatDataSI8
	#define SaMin MinSI8
	#define SaMax MaxSI8
	#define SaMean MeanSI8
	#define SaRootMeanSquare RootMeanSquareSI8
	#define SaMeanCrossing MeanCrossingSI8
	#define SaNegMeanCrossing NegMeanCrossingSI8
	#define SaPosMeanCrossing PosMeanCrossingSI8
	#define SaVariance VarianceSI8
	#define SaPetrosianFractalDimension PetrosianFractalDimensionSI8
	#define SaSkewness SkewnessSI8
	#define SaKurtosis KurtosisSI8
	#define SaAmplitudeGlobalP2pLowFrequency AmplitudeGlobalP2pLowFrequencySI8
	#define SaAmplitudeGlobalP2pHighFrequency AmplitudeGlobalP2pHighFrequencySI8
	#define SaFhtPower0 FhtPower0SI8
	#define SaFhtPower1 FhtPower1SI8
	#define SaFhtPower2 FhtPower2SI8
	#define SaFhtFreq0 FhtFreq0SI8
	#define SaFhtFreq1 FhtFreq1SI8
	#define SaFhtFreq2 FhtFreq2SI8

#elif SA_PRECISION == 8

	#include "DSPUI8.h"

	#define SaInitLib InitLibUI8
	#define SaPrecalcStatData SaPrecalcStatDataUI8
	#define SaMin MinUI8
	#define SaMax MaxUI8
	#define SaMean MeanUI8
	#define SaRootMeanSquare RootMeanSquareUI8
	#define SaMeanCrossing MeanCrossingUI8
	#define SaNegMeanCrossing NegMeanCrossingUI8
	#define SaPosMeanCrossing PosMeanCrossingUI8
	#define SaVariance VarianceUI8
	#define SaPetrosianFractalDimension PetrosianFractalDimensionUI8
	#define SaSkewness SkewnessUI8
	#define SaKurtosis KurtosisUI8
	#define SaAmplitudeGlobalP2pLowFrequency AmplitudeGlobalP2pLowFrequencyUI8
	#define SaAmplitudeGlobalP2pHighFrequency AmplitudeGlobalP2pHighFrequencyUI8
	#define SaFhtPower0 FhtPower0UI8
	#define SaFhtPower1 FhtPower1UI8
	#define SaFhtPower2 FhtPower2UI8
	#define SaFhtFreq0 FhtFreq0UI8
	#define SaFhtFreq1 FhtFreq1UI8
	#define SaFhtFreq2 FhtFreq2UI8

#elif SA_PRECISION == 15

	#include "DSPSI16.h"

	#define SaInitLib()
	#define SaPrecalcStatData SaPrecalcStatDataSI16
	#define SaMin MinSI16
	#define SaMax MaxSI16
	#define SaMean MeanSI16
	#define SaRootMeanSquare RootMeanSquareSI16
	#define SaMeanCrossing MeanCrossingSI16
	#define SaNegMeanCrossing NegMeanCrossingSI16
	#define SaPosMeanCrossing PosMeanCrossingSI16
	#define SaVariance VarianceSI16
	#define SaPetrosianFractalDimension PetrosianFractalDimensionSI16
	#define SaSkewness SkewnessSI16
	#define SaKurtosis KurtosisSI16
	#define SaAmplitudeGlobalP2pLowFrequency AmplitudeGlobalP2pLowFrequencySI16
	#define SaAmplitudeGlobalP2pHighFrequency AmplitudeGlobalP2pHighFrequencySI16
	#define SaFhtPower0 FhtPower0SI16
	#define SaFhtPower1 FhtPower1SI16
	#define SaFhtPower2 FhtPower2SI16
	#define SaFhtFreq0 FhtFreq0SI16
	#define SaFhtFreq1 FhtFreq1SI16
	#define SaFhtFreq2 FhtFreq2SI16

#elif SA_PRECISION == 16

	#include "DSPUI16.h"

	#define SaInitLib InitLibUI16
	#define SaPrecalcStatData SaPrecalcStatDataUI16
	#define SaMin MinUI16
	#define SaMax MaxUI16
	#define SaMean MeanUI16
	#define SaRootMeanSquare RootMeanSquareUI16
	#define SaMeanCrossing MeanCrossingUI16
	#define SaNegMeanCrossing NegMeanCrossingUI16
	#define SaPosMeanCrossing PosMeanCrossingUI16
	#define SaVariance VarianceUI16
	#define SaPetrosianFractalDimension PetrosianFractalDimensionUI16
	#define SaSkewness SkewnessUI16
	#define SaKurtosis KurtosisUI16
	#define SaAmplitudeGlobalP2pLowFrequency AmplitudeGlobalP2pLowFrequencyUI16
	#define SaAmplitudeGlobalP2pHighFrequency AmplitudeGlobalP2pHighFrequencyUI16
	#define SaFhtPower0 FhtPower0UI16
	#define SaFhtPower1 FhtPower1UI16
	#define SaFhtPower2 FhtPower2UI16
	#define SaFhtFreq0 FhtFreq0UI16
	#define SaFhtFreq1 FhtFreq1UI16
	#define SaFhtFreq2 FhtFreq2UI16

#else

	#error Unsupported SA_PRECISION value

#endif

#endif //(NEUTON_MODEL_EXTRACTED_FEATURES_COUNT > 0)

#endif // NEUTON_FE_STATISTICAL_DSP_H
