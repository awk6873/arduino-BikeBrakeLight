/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_POSTPROCESSING_BLOCKS_MOVING_AVERAGE_H
#define NEUTON_POSTPROCESSING_BLOCKS_MOVING_AVERAGE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

///
/// \brief Structure descibing moving_average postprocessing block
///
typedef struct
{
	///
	/// \brief Pointer to window buffer
	///
	float* windowBuffer;

	///
	/// \brief Pointer to average values
	///
	float* averages;
	
	///
	/// \brief Threshold value
	///
	float threshold;

	///
	/// \brief Window buffer size in samples
	///
	uint32_t windowSize;

	///
	/// \brief Elements count in predictions array
	///
	uint16_t elementsCount;

	///
	/// \brief Current index in window buffer
	///
	uint16_t currentIndex;

	///
	/// \brief Count of predictions to be suppressed after successfull result
	///
	uint32_t suppressionCount;

	///
	/// \brief Current suppression counter
	///
	uint32_t suppressionCurrentCounter;

	///
	/// \brief Flag indicating first run
	///
	uint16_t isInitialized;
}
neuton_postprocessing_block_moving_average_instance;

///
/// \brief Process new data buffer
/// \details This function collect window buffer, average predictions and find maximum probability index
/// \param instance - pointer to movign average postprocessing block
/// \param data - pointer to predictions buffer
/// \param output - pointer to return average values
/// \param index - pointer to return index of maximum probability index
/// \return Return 0 if average values are ready, otherwise 1
///
int8_t NeutonPostprocessingBlockMovingAverageProcess(
	neuton_postprocessing_block_moving_average_instance *instance,
	const float *data, float **output, uint16_t *index);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // NEUTON_POSTPROCESSING_BLOCKS_MOVING_AVERAGE_H
