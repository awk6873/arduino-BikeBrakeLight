/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef NEUTON_DSP_FHT_INIT_CONST_H
#define NEUTON_DSP_FHT_INIT_CONST_H

#ifndef PYTHON_LIB

// Must work with generated model.h and config files

#include "fhtInstance.h"

// located in model folder (folder must be provided in 'include folders')
#include "../../model/config.h"

#ifdef __cplusplus
extern "C" {
#endif

///
/// \brief Initializes FHT with const data from generated config file
/// \param fht - pointer to FHT block
///
void FhtInstanceInitByConstData(neuton_dsp_fht_instance* fht);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // PYTHON_LIB

#endif //NEUTON_DSP_FHT_INIT_CONST_H
