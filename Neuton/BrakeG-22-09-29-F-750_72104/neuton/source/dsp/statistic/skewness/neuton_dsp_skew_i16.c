#include <neuton/dsp/neuton_dsp_statistic.h>
#include <neuton/dsp/neuton_dsp_fast_math.h>
#include <neuton/dsp/support/neuton_f24.h>
#include <neuton/private/common/neuton_mem.h>

// ///////////////////////////////////////////////////////////////////////////

#define CTX_NOT_AVAILABLE        (0U)
#define SUM_AND_VAR_AVAILABLE    (neuton_u8_t)(NEUTON_DSP_STAT_CTX_SUM_BIT | NEUTON_DSP_STAT_CTX_VAR_BIT)
#define SUM_AVAILABLE            (neuton_u8_t)(NEUTON_DSP_STAT_CTX_SUM_BIT)
#define VAR_AVAILABLE            (neuton_u8_t)(NEUTON_DSP_STAT_CTX_VAR_BIT)
#define SKEW_REQUIRED_EXP        (-12)

// ///////////////////////////////////////////////////////////////////////////

static neuton_i32_t f24_moments_to_skew_(neuton_f24_t m2, neuton_f24_t m3)
{
    neuton_f24_t std   = f24_sqrt(m2);
    neuton_f24_t denom = f24_mul(std, std);
    denom = f24_mul(denom, std);

    neuton_f24_t skew = f24_div(m3, denom);

    return (neuton_i32_t)f24_to_i32(skew, SKEW_REQUIRED_EXP) + NEUTON_INT16_MAX;
}

// ///////////////////////////////////////////////////////////////////////////

static neuton_i32_t skew_and_var_(const neuton_i16_t* p_input, neuton_u16_t num,
                                neuton_dsp_stat_ctx_i16_t* p_ctx, const neuton_i16_t mean)
{
    neuton_u8_t sign;
    neuton_i8_t shift;
    neuton_i8_t result_exp;
    neuton_i8_t hbit_mult;
    neuton_i32_t diff;
    neuton_u16_t loop_cnt;
    neuton_u32_t pow_diff;

    neuton_f24_t m2 = f24_init(0, 0, 0);
    neuton_f24_t m3 = f24_init(0, 0, 0);
    neuton_f24_t n_inputs = f24_init(num, 0, 0);
    const neuton_u8_t* HIGH_BIT_HELPER = neuton_f24_get_highbit_helper();

    loop_cnt = num;

    while (loop_cnt > 0U)
    {
        diff = *p_input++ - mean;

        sign = ((diff >> 25) & NEUTON_F24_SIGNED);
        diff = abs(diff);

        shift = 8 + HIGH_BIT_HELPER[diff >> 8];
        if (shift == 8)
            shift = HIGH_BIT_HELPER[diff];

        hbit_mult = 16 - shift;
        pow_diff = diff << hbit_mult;
        result_exp = -hbit_mult;

        pow_diff *= diff;
        pow_diff >>= shift;
        result_exp += shift;

        m2 = f24_addx(pow_diff, result_exp, 0, m2.man, f24_get_exp(m2), NEUTON_F24_SIGN(m2)); 

        pow_diff *= diff;
        pow_diff >>= shift;
        result_exp += shift;

        m3 = f24_addx(pow_diff, result_exp, sign, m3.man, f24_get_exp(m3), NEUTON_F24_SIGN(m3));

        loop_cnt--;
    }

    m2 = f24_div(m2, n_inputs);
    m3 = f24_div(m3, n_inputs);

    NEUTON_DSP_RETURN_IF(m2.man == 0, NEUTON_DSP_ZERO_DIV_FLAG);

    if (p_ctx)
    {
        p_ctx->flags.is.var = true;
        p_ctx->value.var = (neuton_i16_t)f24_to_i32(m2, 0);
    }

    return f24_moments_to_skew_(m2, m3);
}

// ///////////////////////////////////////////////////////////////////////////

static neuton_i32_t skew_(const neuton_i16_t* p_input, neuton_u16_t num,
                           const neuton_i16_t mean, const neuton_i32_t var)
{
    NEUTON_DSP_RETURN_IF(var == 0, NEUTON_DSP_ZERO_DIV_FLAG);

    neuton_u8_t sign;
    neuton_i8_t shift;
    neuton_i8_t result_exp;
    neuton_i8_t hbit_mult;
    neuton_i32_t diff;
    neuton_u16_t loop_cnt;
    neuton_u32_t pow_diff;

    neuton_f24_t m3      = f24_init(0, 0, 0);
    neuton_f24_t std     = f24_init(neuton_dsp_sqrt_u32(var), 0, 0);
    const neuton_u8_t* HIGH_BIT_HELPER = neuton_f24_get_highbit_helper();

    loop_cnt = num;

    while (loop_cnt > 0U)
    {
        diff = *p_input++ - mean;

        sign = ((diff >> 25) & NEUTON_F24_SIGNED);
        diff = abs(diff);

        shift = 8 + HIGH_BIT_HELPER[diff >> 8];
        if (shift == 8)
            shift = HIGH_BIT_HELPER[diff];

        hbit_mult = 16 - shift;
        pow_diff = diff << hbit_mult;
        result_exp = -hbit_mult;

        pow_diff *= diff;
        pow_diff >>= shift;

        pow_diff *= diff;
        pow_diff >>= shift;
        result_exp +=  (2 * shift);

        m3 = f24_addx(pow_diff, result_exp, sign, m3.man, f24_get_exp(m3), NEUTON_F24_SIGN(m3));

        loop_cnt--;
    }

    m3 = f24_div(m3, f24_init(num, 0, 0));

    neuton_f24_t denom = f24_mul(std, std);
    denom = f24_mul(denom, std);

    neuton_f24_t skew = f24_div(m3, denom);

    return (neuton_i32_t)f24_to_i32(skew, SKEW_REQUIRED_EXP) + NEUTON_INT16_MAX;
}

// ///////////////////////////////////////////////////////////////////////////

neuton_i32_t neuton_dsp_skew_i16(const neuton_i16_t* p_input, neuton_u16_t num,
                                neuton_dsp_stat_ctx_i16_t* p_ctx)
{
    neuton_i16_t mean;
    neuton_i32_t skew;
    neuton_u8_t f;

    f = (p_ctx == NULL) ? CTX_NOT_AVAILABLE : (p_ctx->flags.all & SUM_AND_VAR_AVAILABLE);

    switch (f)
    {
    case CTX_NOT_AVAILABLE:
        mean = neuton_dsp_sum_i16(p_input, num, p_ctx) / num;
        skew = skew_and_var_(p_input, num, p_ctx, mean);
        break;
    case SUM_AND_VAR_AVAILABLE:
        mean = p_ctx->value.sum / num;
        skew = skew_(p_input, num, mean, p_ctx->value.var);
        break;
    case SUM_AVAILABLE:
        mean = p_ctx->value.sum / num;
        skew = skew_and_var_(p_input, num, p_ctx, mean);
        break;
    case VAR_AVAILABLE:
        mean = neuton_dsp_sum_i16(p_input, num, p_ctx) / num;
        skew = skew_(p_input, num, mean, p_ctx->value.var);
        break;
    default:
        skew = NEUTON_INT32_MIN; // should never get here
        break;
    }

    return skew;        
}