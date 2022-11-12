#include "neuton_kws_frontend.h"

#include <neuton/dsp/support/neuton_dsp_windowing.h>
#include <neuton/dsp/statistic/neuton_dsp_min_max.h>
#include <neuton/dsp/support/neuton_dsp_scale_minmax.h>
#include <neuton/dsp/support/neuton_dsp_clipping.h>

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

// ///////////////////////////////////////////////////////////////////////////

#define BITS_IN_SAMPLE (sizeof(audio_sample_t) * 8)

#ifdef IS_INT_AUDIO_DATA
#define SCALE_SAMPLE(x)             (((neuton_f32_t) x) / ((neuton_u32_t)(1) << (BITS_IN_SAMPLE - 1)))
#else
#define SCALE_SAMPLE(x)             (x)
#endif

#ifndef M_PI
#define M_PI       3.14159265358979323846   // pi
#endif

// ///////////////////////////////////////////////////////////////////////////

static void rfht_init_tables_(neuton_dsp_rfht_instance_t* p_ctx)
{
    for (neuton_u16_t index = 0; index < p_ctx->window_size; ++index)
    {
        neuton_u16_t reversed_index = 0;
        for (neuton_u16_t j = 0; j < p_ctx->log_n; ++j)
        {
            if (index & (1 << j))
            {
                reversed_index |= 1 << (p_ctx->log_n - 1 - j);
            }
        }
        p_ctx->p_rev_bit_index[index] = reversed_index;
    }

    neuton_f32_t* p_sin_table = (neuton_f32_t*)p_ctx->p_sin_table;
    neuton_f32_t* p_tg_table = (neuton_f32_t*)p_ctx->p_tg_table;
    p_sin_table[0] = 0.f;
    p_tg_table[0] = 0.f;
    for (uint16_t k = 1; k < (p_ctx->window_size >> 2); ++k)
    {
        p_sin_table[k] = sinf(2 * M_PI * k / p_ctx->window_size);
        p_tg_table[k] = tanf(M_PI * k / p_ctx->window_size);
    }
}

// ///////////////////////////////////////////////////////////////////////////

static void kws_frontend_(void* p_window, neuton_u16_t window_size,
                            void* p_user_ctx)
{
    neuton_kws_frontend_ctx_t* p_kws = (neuton_kws_frontend_ctx_t*)p_user_ctx;
    neuton_dsp_melspectr_ctx_f32_t* p_melspectr_ctx = &p_kws->melspectr;
    audio_sample_t* audio = (audio_sample_t*)p_window;

    /* Scale audio and apply windowing */
    for (neuton_u32_t i = 0; i < p_melspectr_ctx->fht.window_size; ++i)
        p_kws->p_scaled_audio[i] = SCALE_SAMPLE(audio[i]) * p_kws->p_hanning_window[i];

    /*  Make spectrograms */
    bool ready = (neuton_dsp_melspectr_make_f32(p_melspectr_ctx, p_kws->p_scaled_audio) == 0);

    if (ready)
    {
        neuton_u16_t mels_count = p_melspectr_ctx->time_bands * p_melspectr_ctx->freq_bands;

        if (p_kws->p_normalized_spectrum)
        {
            /* Find min & max value in mel spectrum */
            neuton_f32_t min, max;
            neuton_dsp_min_max_f32(p_melspectr_ctx->p_melspectrum, mels_count, &min, &max);

            /* Normalize mel spectrum */
            neuton_dsp_scale_minmax_f32(p_melspectr_ctx->p_melspectrum, mels_count, min, max, 
                                        p_kws->p_normalized_spectrum);
        }

        /* Call callback function (inference) */
        neuton_f32_t* p_spectrum = p_kws->p_normalized_spectrum ? p_kws->p_normalized_spectrum : p_melspectr_ctx->p_melspectrum;

        if (p_kws->on_spectrum_ready_cb)
            p_kws->on_spectrum_ready_cb(p_kws, p_spectrum);

        /* Shift mel spectrum */
        neuton_dsp_melspectr_shift_f32(p_melspectr_ctx, p_kws->melspectr_shift);
    }
}

// ///////////////////////////////////////////////////////////////////////////

neuton_status_t neuton_kws_frontend_init(neuton_kws_frontend_ctx_t* p_ctx, neuton_u16_t window_size, 
                                neuton_u16_t window_hop, neuton_u16_t sample_rate, 
                                neuton_u16_t melspectr_count, neuton_u16_t melspectr_nmels, 
                                neuton_u16_t melspectr_shift, neuton_f32_t* p_normalized_spectrum, 
                                neuton_kws_on_spectrum_ready_cb_t cb, void* p_user_ctx)
{
    memset(p_ctx, 0, sizeof(*p_ctx));

    /*
     * Configure keyword spotting p_ctx
     */
    p_ctx->melspectr_shift       = melspectr_shift;
    p_ctx->p_scaled_audio        = malloc(sizeof(neuton_f32_t) * window_size);
    p_ctx->p_normalized_spectrum = p_normalized_spectrum;
    p_ctx->on_spectrum_ready_cb  = cb;
    p_ctx->p_user_ctx            = p_user_ctx;

    /*
     * Configure sliding window
     */
    audio_sample_t* p_window = (audio_sample_t*)malloc(sizeof(audio_sample_t) * window_size);

    neuton_sliding_window_init(&p_ctx->sliding_window, p_window, window_size, 
                                sizeof(audio_sample_t), window_hop, kws_frontend_, p_ctx);

    /*
     * Configure Mel-spectrogram context
     */
    neuton_dsp_melspectr_ctx_f32_t* p_melspectr_ctx = &p_ctx->melspectr;

    p_melspectr_ctx->sample_rate    = sample_rate;
    p_melspectr_ctx->time_bands     = melspectr_count;
    p_melspectr_ctx->freq_bands     = melspectr_nmels;
    p_melspectr_ctx->p_melspectrum  = malloc(sizeof(neuton_f32_t) * melspectr_count * melspectr_nmels);

    /*
     * Configure FHT
     */
    neuton_dsp_rfht_instance_t* p_fht = &p_melspectr_ctx->fht;

    p_fht->log_n            = (neuton_u16_t)log2f(window_size);
    p_fht->window_size      = window_size;
    p_fht->p_rev_bit_index  = malloc(sizeof(neuton_u16_t) * window_size);
    p_fht->p_sin_table      = malloc(sizeof(neuton_f32_t) * (window_size / 4));
    p_fht->p_tg_table       = malloc(sizeof(neuton_f32_t) * (window_size / 4));

    rfht_init_tables_(p_fht);
    
    /*
     * Prepare Hanning window
     */
    p_ctx->p_hanning_window = malloc(sizeof(neuton_f32_t) * window_size);

    neuton_dsp_window_hanning_f32(p_ctx->p_hanning_window, window_size);

    return NEUTON_STATUS_SUCCESS;
}

// ///////////////////////////////////////////////////////////////////////////

void neuton_kws_frontend_destroy(neuton_kws_frontend_ctx_t* p_ctx)
{
    if (p_ctx->p_scaled_audio != NULL)
    {
        free(p_ctx->p_scaled_audio);
        p_ctx->p_scaled_audio = NULL;
    }

    if (p_ctx->p_hanning_window != NULL)
    {
        free(p_ctx->p_hanning_window);
        p_ctx->p_hanning_window = NULL;
    }

    if (p_ctx->sliding_window.p_window != NULL)
    {
        free(p_ctx->sliding_window.p_window);
        p_ctx->sliding_window.p_window = NULL;
    }

    if (p_ctx->melspectr.p_melspectrum != NULL)
    {
        free(p_ctx->melspectr.p_melspectrum);
        p_ctx->melspectr.p_melspectrum = NULL;
    }

    if (p_ctx->melspectr.fht.p_sin_table != NULL)
    {
        free(p_ctx->melspectr.fht.p_sin_table);
        p_ctx->melspectr.fht.p_sin_table = NULL;
    }

    if (p_ctx->melspectr.fht.p_tg_table != NULL)
    {
        free(p_ctx->melspectr.fht.p_tg_table);
        p_ctx->melspectr.fht.p_tg_table = NULL;
    }

    if (p_ctx->melspectr.fht.p_rev_bit_index!= NULL)
    {
        free(p_ctx->melspectr.fht.p_rev_bit_index);
        p_ctx->melspectr.fht.p_rev_bit_index = NULL;
    }
}

// ///////////////////////////////////////////////////////////////////////////

void neuton_kws_frontend_process(neuton_kws_frontend_ctx_t* p_ctx, const audio_sample_t* p_samples, 
                            neuton_u32_t samples_num)
{
    neuton_sliding_window_feed(&p_ctx->sliding_window, (void*)p_samples, samples_num);
}