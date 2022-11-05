#ifndef _NEUTON_NN_DATA_PREPARATION_HELPER_H_
#define _NEUTON_NN_DATA_PREPARATION_HELPER_H_

#include <neuton_generated/neuton_user_types.h>
#include <neuton/utils/neuton_ordered_window.h>

#ifdef   __cplusplus
extern "C"
{
#endif

/**
 * @brief Get pointer to array of input usage masks
 */
const neuton_u8_t* neuton_nn_helper_get_input_usage_mask(void);

/**
 * @brief Extract DSP features from the input data window
 * 
 * @param[in] p_input_window    Input data window
 * @param[in] p_ctx             Input window context
 * @param[out] p_features       Output features buffer
 * 
 * @return neuton_u16_t Number of features extracted to the p_features buffer
 */
neuton_u16_t neuton_nn_helper_extract_dsp_features(const neuton_input_t* p_input_window,
                                                neuton_ordered_window_ctx_t* p_ctx,
                                                neuton_feature_t* p_features);

/**
 * @brief Scale extracted DSP features
 * 
 * @param[in, out]  p_features    DSP features to be scaled
 * @param[in]       num           Number of DSP features
 */
void neuton_nn_helper_scale_dsp_features(neuton_feature_t* p_features, neuton_u16_t num);

/**
 * @brief Scale LAG features
 * 
 * @param[in, out]  p_input_window       Input window with LAG features
 * @param[in]       p_ctx                Input window context
 * @param[in]       lag_features_num     Number of input(LAG) features to be scaled
 */
void neuton_nn_helper_scale_lag_features(neuton_input_t* p_input_window,
                                        neuton_ordered_window_ctx_t* p_ctx,
                                        neuton_u16_t lag_features_num);

/**
 * @brief Scale original input features
 * 
 * @param[in, out]  p_input     Input features to be scaled
 * @param[in]       num         Number of input features 
 */
void neuton_nn_helper_scale_inputs(neuton_input_t* p_input, neuton_u16_t num);

/**
 * @brief Get number of input features scaling factors
 */
neuton_u16_t neuton_nn_helper_input_scaling_num(void);

#ifdef   __cplusplus
}
#endif

#endif /* _NEUTON_NN_DATA_PREPARATION_HELPER_H_ */