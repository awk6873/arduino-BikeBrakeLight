/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_DSP_FHT_F32_H
#define NEUTON_DSP_FHT_F32_H

#include "fhtInstance.h"

#ifdef __cplusplus
extern "C" {
#endif

///
/// \brief Make FHT on input data
/// \param instance - pointer to FHT block
///
void NeutonDspFhtMakeF32(const neuton_dsp_fht_instance* instance);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // NEUTON_DSP_FHT_F32_H
