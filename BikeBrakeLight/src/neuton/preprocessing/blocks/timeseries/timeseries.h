/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_PREPROCESSING_BLOCKS_TIMESERIES_H
#define NEUTON_PREPROCESSING_BLOCKS_TIMESERIES_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

///
/// \brief Callback function. Called when window buffer is ready
/// \param ctx - pointer to timeseries preprocessing block
/// \param data - pointer to window buffer
///
typedef void (*neuton_preprocessing_block_timeseries_on_window_cb)(void* ctx, void* data);

///
/// \brief Structure descibing timeseries preprocessing block
///
typedef struct
{
	///
	/// \brief Pointer to window buffer
	///
	void* windowBuffer;

	///
	/// \brief Callback function
	///
	neuton_preprocessing_block_timeseries_on_window_cb onWindow;

	///
	/// \brief Current window buffer fill
	///
	uint32_t currentFill;

	///
	/// \brief Sample size in bytes
	///
	uint16_t dataStride;

	///
	/// \brief Window buffer size in samples
	///
	uint16_t windowSize;

	///
	/// \brief Window hop in samples. Must be less or equal @window_size
	///
	uint16_t windowHop;

	///
	/// \brief Pointer to store user context. Can be accessed in callback function
	///
	void* userData;
}
neuton_preprocessing_block_timeseries_instance;

///
/// \brief Process new data buffer
/// \details This function collect window buffer, perform data shift according to window_hop
/// parameter and call callback function
/// \param instance - pointer to timeseries preprocessing block
/// \param data - pointer to data buffer
/// \param elementsCount - samples count in data buffer
///
void NeutonPreprocessingBlockTimeseriesProcess(
	neuton_preprocessing_block_timeseries_instance *instance, const void *data, uint32_t elementsCount);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // NEUTON_PREPROCESSING_BLOCKS_TIMESERIES_H
