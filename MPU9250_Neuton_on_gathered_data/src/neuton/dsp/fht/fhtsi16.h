/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_DSP_FHT_SI16_H
#define NEUTON_DSP_FHT_SI16_H

#include "fhtInstance.h"

#define FRACT_BITS_15 15

#ifdef __cplusplus
extern "C" {
#endif

///
/// \brief Make FHT on input data
/// \param instance - pointer to FHT block
///
uint16_t NeutonDspFhtMake16(const neuton_dsp_fht_instance* instance);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // NEUTON_DSP_FHT_SI16_H
