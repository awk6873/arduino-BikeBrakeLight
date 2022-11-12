#ifndef _NEUTON_EXAMPLES_KEYWORD_SPOTTING_FRONTEND_H_
#define _NEUTON_EXAMPLES_KEYWORD_SPOTTING_FRONTEND_H_

#include <neuton/neuton_types.h>
#include <neuton/dsp/transform/neuton_dsp_rfht.h>
#include <neuton/dsp/transform/neuton_dsp_melspectr.h>
#include <neuton/utils/neuton_sliding_window.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Type of audio sample */
#ifdef IS_INT_AUDIO_DATA
typedef int16_t audio_sample_t;
#else
typedef neuton_f32_t audio_sample_t;
#endif // IS_INT_AUDIO_DATA

/**
 * @brief Callback function. Called when mel-spectrogram is ready
 * 
 * @param p_ctx         Pointer to user context
 * @param p_spectrum    Pointer to mel-spectrogram
 */
typedef void (*neuton_kws_on_spectrum_ready_cb_t)(void* p_ctx, neuton_f32_t* p_spectrum);

/**
 * @brief Keyword spotting context
 * 
 */
typedef struct
{
    /** Sliding window context */
    neuton_sliding_window_ctx_t sliding_window;

    /** Mel-spectrogram context */
    neuton_dsp_melspectr_ctx_f32_t melspectr;

    /** On mel-spectrogrom ready callback */
    neuton_kws_on_spectrum_ready_cb_t on_spectrum_ready_cb;

    /** Buffer to store scaled audio */
    neuton_f32_t* p_scaled_audio;

    /** Buffer to store Hanning window */
    neuton_f32_t* p_hanning_window;

    /** Buffer to store normalized spectrogram */
    neuton_f32_t* p_normalized_spectrum;

    /** Number of columns to shift in mel-spectrogram */
    neuton_u16_t melspectr_shift;

    /** Pointer to the user context. Can be accessed in callback function */
    void* p_user_ctx;
} neuton_kws_frontend_ctx_t;

/**
 * @brief Initialize keyword spotting context
 * 
 * @param[out] p_ctx                Pointer to keyword spotting context
 * @param[in] window_size           Window size, in samples
 * @param[in] window_hop            Window hop, in samples
 * @param[in] sample_rate           Sample rate of signal
 * @param[in] melspectr_count       Mel-spectrogram count
 * @param[in] melspectr_nmels       Count of MEL frequency bins
 * @param[in] melspectr_shift       Count of Mel-spectrograms to shift
 * @param[in] p_normalized_spectrum Pointer to normalized spetrogram
 * @param[in] cb                    Callback function. Called when mel-spectrogram is ready
 * @param[in] p_user_ctx            Pointer to the user context. Can be accessed in callback function
 * 
 * @return @ref neuton_status_t 
 */
neuton_status_t neuton_kws_frontend_init(neuton_kws_frontend_ctx_t* p_ctx, neuton_u16_t window_size, 
                                        neuton_u16_t window_hop, neuton_u16_t sample_rate, 
                                        neuton_u16_t melspectr_count, neuton_u16_t melspectr_nmels, 
                                        neuton_u16_t melspectr_shift, neuton_f32_t* p_normalized_spectrum, 
                                        neuton_kws_on_spectrum_ready_cb_t cb, void* p_user_ctx);

/**
 * @brief Destroy keyword spotting context, only deallocates internal objects, 
 *      context of 'neuton_kws_frontend_ctx_t' object must be deallocated by the user)
 * 
 * @param[in] p_ctx Pointer to keyword spotting context
 */
void neuton_kws_frontend_destroy(neuton_kws_frontend_ctx_t* p_ctx);

/**
 * @brief Process new audio data buffer
 * 
 * @param[in] p_ctx         Pointer to keyword spotting context
 * @param[in] p_samples     Audio samples buffer
 * @param[in] samples_num   Number of audio samples in buffer
 */
void neuton_kws_frontend_process(neuton_kws_frontend_ctx_t* p_ctx, const audio_sample_t* p_samples, 
                                neuton_u32_t samples_num);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif // _NEUTON_EXAMPLES_KEYWORD_SPOTTING_FRONTEND_H_
