/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_DSP_FHT_INSTANCE_H
#define NEUTON_DSP_FHT_INSTANCE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

///
/// \brief Structure describing FHT block
///
typedef struct _neuton_dsp_fht_instance
{
	///
	/// \brief FHT sin() table (elements count is @fftWindowSize / 4). Initialize with @fftSinTable
	///
	void* sinTable;

	///
	/// \brief FHT tg() table (elements count is @fftWindowSize / 4). Initialize with @fftTgTable
	///
	void* tgTable;

	///
	/// \brief FHT reordering array (elements count is @fftWindowSize). Initialize with @fftReversedBitIndexArray
	///
	uint16_t* reversedBitIndexArray;

	///
	/// \brief FHT Window buffer array (elements count is @fftWindowSize). Initialize with @fftWindow
	///
	void* windowArray;

	///
	/// \brief FHT window size (n elements)
	///
	uint16_t fhtWindowSize;

	///
	/// \brief Cached value of log2(@n_fht)
	///
	uint16_t logN;

} neuton_dsp_fht_instance;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // NEUTON_DSP_FHT_INSTANCE_H
