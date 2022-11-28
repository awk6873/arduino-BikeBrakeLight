#include <neuton/dsp/neuton_dsp_statistic.h>
#include <neuton/private/common/neuton_mem.h>

// ///////////////////////////////////////////////////////////////////////////

neuton_i16_t neuton_dsp_tcr_i8(const neuton_i8_t* p_input, neuton_u16_t num,
                                const neuton_i8_t threshold)
{
    neuton_u16_t loop_cnt;
    neuton_i16_t tcr = 0U;

    bool sign_val;
    bool sign_next_val;

#if (NEUTON_USE_LOOPUNROLL == 1)
    sign_next_val = NEUTON_INT8_CHECK_SIGN(*p_input - threshold);

    neuton_i32_t *p_input_i32 = (neuton_i32_t *)p_input;
    neuton_i32_t i8x4;

    /* Loop unrolling: Compute 4 outputs at a time */
    loop_cnt = num >> 2U;

    while (loop_cnt > 0U)
    {
        i8x4 = *p_input_i32++;

        sign_val = NEUTON_INT8_CHECK_SIGN(((i8x4 << 24U) >> 24U) - threshold);

        if (sign_val != sign_next_val)
            tcr++;

        sign_next_val = NEUTON_INT8_CHECK_SIGN(((i8x4 << 16U) >> 24U) - threshold);

        if (sign_val != sign_next_val)
            tcr++;

        sign_val = NEUTON_INT8_CHECK_SIGN(((i8x4 << 8U) >> 24U) - threshold);

        if (sign_val != sign_next_val)
            tcr++;

        sign_next_val = NEUTON_INT8_CHECK_SIGN((i8x4 >> 24U) - threshold);

        if (sign_val != sign_next_val)
            tcr++;

        loop_cnt--;
    }

    /* Loop unrolling: Compute remaining outputs */
    loop_cnt = num % 0x4U;
    p_input  = (neuton_i8_t*)p_input_i32;

    sign_val = sign_next_val;
#else

    loop_cnt = num - 1;
    sign_val = NEUTON_INT8_CHECK_SIGN(*p_input++ - threshold);

#endif /* #if (NEUTON_USE_LOOPUNROLL == 1) */

    while (loop_cnt > 0U)
    {
        sign_next_val = NEUTON_INT8_CHECK_SIGN(*p_input++ - threshold);

        if (sign_val != sign_next_val)
            tcr++;

        sign_val = sign_next_val;

        loop_cnt--;
    }

    return (neuton_i16_t)(((int32_t)tcr * NEUTON_INT_TO_F32_PRECISION_FACTOR) / (num - 1));
}

// ///////////////////////////////////////////////////////////////////////////

neuton_i16_t neuton_dsp_tcr_i8_s(const neuton_i8_t* p_input, neuton_u16_t num,
                                size_t stride, const neuton_i8_t threshold)
{
    neuton_u16_t loop_cnt;
    neuton_i16_t tcr = 0U;
    neuton_u16_t i = 0U;

    bool sign_val;
    bool sign_next_val;

#if (NEUTON_USE_LOOPUNROLL == 1)
    /* Loop unrolling: Compute 4 outputs at a time */
    loop_cnt = num >> 2U;

    sign_val = NEUTON_INT8_CHECK_SIGN(p_input[i * stride] - threshold);

    while (loop_cnt > 0U)
    {
        sign_next_val = NEUTON_INT8_CHECK_SIGN(p_input[i * stride] - threshold);

        if (sign_val != sign_next_val)
            tcr++;
        i++;

        sign_val = NEUTON_INT8_CHECK_SIGN(p_input[i * stride] - threshold);

        if (sign_val != sign_next_val)
            tcr++;
        i++;

        sign_next_val = NEUTON_INT8_CHECK_SIGN(p_input[i * stride] - threshold);

        if (sign_val != sign_next_val)
            tcr++;
        i++;

        sign_val = NEUTON_INT8_CHECK_SIGN(p_input[i * stride] - threshold);

        if (sign_val != sign_next_val)
            tcr++;
        i++;

        loop_cnt--;
    }

    /* Loop unrolling: Compute remaining outputs */
    loop_cnt = num % 0x4U;

#else

    loop_cnt = num - 1;
    sign_val = NEUTON_INT8_CHECK_SIGN(p_input[i * stride] - threshold);
    i++;

#endif /* #if (NEUTON_USE_LOOPUNROLL == 1) */

    while (loop_cnt > 0U)
    {
        sign_next_val = NEUTON_INT8_CHECK_SIGN(p_input[i * stride] - threshold);

        if (sign_val != sign_next_val)
            tcr++;
        i++;

        sign_val = sign_next_val;

        loop_cnt--;
    }

    return (neuton_i16_t)(((int32_t)tcr * NEUTON_INT_TO_F32_PRECISION_FACTOR) / (num - 1));
}
