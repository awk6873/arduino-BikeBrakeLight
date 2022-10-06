/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "fhtInitConst.h"

#ifndef PYTHON_LIB

void FhtInstanceInitByConstData(neuton_dsp_fht_instance* fht)
{
#if (CONTAINS_FFT_FEATURES > 0)

	fht->fhtWindowSize = fftWindowSize;
	fht->logN = fftLogN;
	fht->reversedBitIndexArray = (uint16_t*)fftReversedBitIndexArray;
	fht->sinTable = (input_t*)fftSinTable;
	fht->tgTable = (input_t*)fftTgTable;
	fht->windowArray = fftWindow;

#endif
}

#endif // PYTHON_LIB
