#ifndef _NEUTON_DSP_TYPES_H_
#define _NEUTON_DSP_TYPES_H_

#include <neuton/neuton_common_types.h>

#ifdef   __cplusplus
extern "C"
{
#endif

/**
 * @brief Reset utility macro for @ref neuton_dsp_stat_ctx_f32_t context structure
 * 
 */
#define NEUTON_DSP_STAT_CTX_RESET(ctx) \
      ctx.flags.all = 0

#define NEUTON_DSP_STAT_CTX_SUM_BIT           (1 << 0)
#define NEUTON_DSP_STAT_CTX_TSS_BIT           (1 << 1)
#define NEUTON_DSP_STAT_CTX_VAR_BIT           (1 << 2)

/**
 * @brief Statistics context flags that indicating the presence of values ​​to use
 * 
 */
typedef union neuton_dsp_stat_ctx_flags_u
{
    struct
    {
      bool sum  : 1;
      bool tss  : 1;
      bool var  : 1;
    } is;
    neuton_u8_t all;
} neuton_dsp_stat_ctx_flags_t;

/**
 * @brief  Floating-point statistics reusable context, in which is used to store derived or intermediate 
 *          metrics to simplify / speed up the calculation of dependent metrics
 */
typedef struct neuton_dsp_stat_ctx_f32_s
{
  struct
  {
    /* Sum of vector elements */
    neuton_f32_t sum;
    /* Total sum of squares of vector elements */
    neuton_f32_t tss;
    /* Variance of vector elements */
    neuton_f32_t var;
  } value;

  neuton_dsp_stat_ctx_flags_t flags;
} neuton_dsp_stat_ctx_f32_t;

/**
 * @brief  INT8 statistics reusable context, in which is used to store derived or intermediate 
 *          metrics to simplify / speed up the calculation of dependent metrics
 */
typedef struct neuton_dsp_stat_ctx_i8_s
{
  struct
  {
    /* Sum of vector elements */
    neuton_i32_t sum;
    /* Total sum of squares of vector elements */
    neuton_u32_t tss;
    /* Variance of vector elements */
    neuton_i16_t var;
  } value;

  neuton_dsp_stat_ctx_flags_t flags;
} neuton_dsp_stat_ctx_i8_t;

/**
 * @brief  INT16 statistics reusable context, in which is used to store derived or intermediate 
 *          metrics to simplify / speed up the calculation of dependent metrics
 */
typedef struct neuton_dsp_stat_ctx_i16_s
{
  struct
  {
    /* Total sum of squares of vector elements */
    neuton_u64_t tss;
    /* Sum of vector elements */
    neuton_i32_t sum;
    /* Variance of vector elements */
    neuton_i32_t var;
  } value;

  neuton_dsp_stat_ctx_flags_t flags;
} neuton_dsp_stat_ctx_i16_t;

/**
 * @brief  INT32 statistics reusable context, in which is used to store derived or intermediate 
 *          metrics to simplify / speed up the calculation of dependent metrics
 */
typedef struct neuton_dsp_stat_ctx_i32_s
{
  struct
  {
    /* Total sum of squares of vector elements */
    neuton_u64_t tss;
    /* Sum of vector elements */
    neuton_i64_t sum;
    /* Variance of vector elements */
    neuton_i64_t var;
  } value;

  neuton_dsp_stat_ctx_flags_t flags;
} neuton_dsp_stat_ctx_i32_t;

/**
 * @brief Possible DSP pipelines for feature extraction
 */
typedef enum neuton_dsp_pipeline_e
{
    /** Statistical features DSP pipeline */
    NEUTON_DSP_PIPELINE_STAT,

    /** Spectral features DSP pipeline */
    NEUTON_DSP_PIPELINE_SPECTRAL,
} neuton_dsp_pipeline_t;

/**
 * @brief Statistical features ID, should be kept in the same order as in neuton_dsp_stat_features_mask_t
 */
typedef enum neuton_dsp_stat_feature_id_e
{
    NEUTON_DSP_STAT_FID_MIN = 0,
    NEUTON_DSP_STAT_FID_MAX,
    NEUTON_DSP_STAT_FID_RANGE,
    NEUTON_DSP_STAT_FID_MEAN,
    NEUTON_DSP_STAT_FID_MAD,
    NEUTON_DSP_STAT_FID_SKEW,
    NEUTON_DSP_STAT_FID_KUR,
    NEUTON_DSP_STAT_FID_STD,
    NEUTON_DSP_STAT_FID_RMS,
    NEUTON_DSP_STAT_FID_MCR,
    NEUTON_DSP_STAT_FID_ZCR,
    NEUTON_DSP_STAT_FID_TCR,
    NEUTON_DSP_STAT_FID_P2P_LF,
    NEUTON_DSP_STAT_FID_P2P_HF,
} neuton_dsp_stat_feature_id_t;

/**
 * @brief Spectral features ID, should be kept in the same order as in neuton_dsp_spectr_features_mask_t
 */
typedef enum neuton_dsp_spectr_feature_id_e
{
    NEUTON_DSP_SPECTR_FID_SPECTRUM = 0,
    NEUTON_DSP_SPECTR_FID_PEAKS_FREQ,
    NEUTON_DSP_SPECTR_FID_PEAKS_AMPL,
} neuton_dsp_spectr_feature_id_t;

/** Spectrum instance block requested for NEUTON_DSP_SPECTR_FID_SPECTRUM as argument */
struct neuton_dsp_rfht_instance_s;
typedef struct neuton_dsp_rfht_instance_s neuton_dsp_arg_fid_spectrum_t;

/**
 * @brief Callback for getting feature argument while feature extraction in DSP pipeline
 * 
 * @param[in] ppln          Pipeline type to resolve feature_id
 * @param[in] feature_id    Feature id for which argument is needed
 * @param[in] args_num      Numbers of arguments to get
 * @param[out] p_args       Pointer to store feature argument, argument type depends on feature_id and input type of data
 */
typedef void (*neuton_dsp_get_feature_arg_cb_t)(neuton_dsp_pipeline_t ppln, neuton_u8_t feature_id,
                                                neuton_u8_t args_num, void* p_args);

#define NEUTON_DSP_STAT_FBIT_MIN          (1U << NEUTON_DSP_STAT_FID_MIN)
#define NEUTON_DSP_STAT_FBIT_MAX          (1U << NEUTON_DSP_STAT_FID_MAX)
#define NEUTON_DSP_STAT_FBIT_RANGE        (1U << NEUTON_DSP_STAT_FID_RANGE)
#define NEUTON_DSP_STAT_FBIT_MEAN         (1U << NEUTON_DSP_STAT_FID_MEAN)
#define NEUTON_DSP_STAT_FBIT_RMS          (1U << NEUTON_DSP_STAT_FID_RMS)
#define NEUTON_DSP_STAT_FBIT_STD          (1U << NEUTON_DSP_STAT_FID_STD)
#define NEUTON_DSP_STAT_FBIT_ZCR          (1U << NEUTON_DSP_STAT_FID_ZCR)
#define NEUTON_DSP_STAT_FBIT_MCR          (1U << NEUTON_DSP_STAT_FID_MCR)
#define NEUTON_DSP_STAT_FBIT_TCR          (1U << NEUTON_DSP_STAT_FID_TCR)
#define NEUTON_DSP_STAT_FBIT_MAD          (1U << NEUTON_DSP_STAT_FID_MAD)
#define NEUTON_DSP_STAT_FBIT_SKEW         (1U << NEUTON_DSP_STAT_FID_SKEW)
#define NEUTON_DSP_STAT_FBIT_KUR          (1U << NEUTON_DSP_STAT_FID_KUR)
#define NEUTON_DSP_STAT_FBIT_P2P_LF       (1U << NEUTON_DSP_STAT_FID_P2P_LF)
#define NEUTON_DSP_STAT_FBIT_P2P_HF       (1U << NEUTON_DSP_STAT_FID_P2P_HF)

/**
 * @brief DSP Statistical features mask type for feature extracting API
 * 
 */
typedef union neuton_dsp_stat_features_mask_u
{
    struct
    {
        bool min    : 1; // Minimum
        bool max    : 1; // Maximum
        bool range  : 1; // Range
        bool mean   : 1; // Mean
        bool mad    : 1; // Mean Absolute Deviation
        bool skew   : 1; // Skewness
        bool kur    : 1; // Kurtosis
        bool std    : 1; // Standard Deviation
        bool rms    : 1; // Root Mean Square
        bool mcr    : 1; // Mean-crossing Rate
        bool zcr    : 1; // Zero-crossing Rate
        bool tcr    : 1; // Threshold-crossing Rate
        bool p2p_lf : 1; // Peak-to-Peak Low Frequency
        bool p2p_hf : 1; // Peak-to-Peak High Frequency
    } is;
    neuton_u16_t all;
} neuton_dsp_stat_features_mask_t;

#define NEUTON_DSP_SPECTR_FBIT_SPECTRUM      (1U << NEUTON_DSP_SPECTR_FID_SPECTRUM)
#define NEUTON_DSP_SPECTR_FBIT_PEAKS_FREQ    (1U << NEUTON_DSP_SPECTR_FID_PEAKS_FREQ)
#define NEUTON_DSP_SPECTR_FBIT_PEAKS_AMPL    (1U << NEUTON_DSP_SPECTR_FID_PEAKS_AMPL)

/**
 * @brief DSP Spectral features mask type for feature extracting API
 * 
 */
typedef union neuton_dsp_spectr_features_mask_u
{   
    struct
    {
        bool spectrum   : 1; // Frequency spectrum @note changing this bit has no effect, enabled by default
        bool peaks_freq : 1; // Peaks frequency indexes in spectrum
        bool peaks_ampl : 1; // Peaks amplitude values in spectrum
    } is;
    neuton_u16_t all;
} neuton_dsp_spectr_features_mask_t;

/**
 * @brief Mask for all available features type for feature extracting API
 * 
 */
typedef union neuton_dsp_features_mask_u
{
    struct
    {
        /** Spectral features mask */
        neuton_dsp_spectr_features_mask_t spectr;

        /** Statistical features mask */
        neuton_dsp_stat_features_mask_t stat;
    } features;
    neuton_u32_t all;
} neuton_dsp_features_mask_t;

#ifdef   __cplusplus
}
#endif

#endif /* _NEUTON_DSP_TYPES_H_ */