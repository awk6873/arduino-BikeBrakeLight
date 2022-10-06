/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#ifndef FLOAT16_H
#define FLOAT16_H

#include <stdint.h>

/**
 * Fast binary sqrt algorithm based on:
 */
#ifdef __cplusplus
extern "C" {
#endif	

/**
 * \brief Float point representation of number
 */
typedef struct
{
	/**
	 * \brief Mantissa of number
	 */
	uint16_t mantissa;
	
	/**
	 * \brief Power of 2
	 */
	uint8_t extExponent;
}
Float16;

void InitF16(Float16* p_Value, uint16_t p_Mantissa, int8_t p_Exponent, uint8_t p_Sign);

uint32_t FixExpUF16(Float16* p_Value, int8_t p_Exponent);

int32_t FixExpF16(Float16* p_Value, int8_t p_Exponent);

void AlignmentF16(Float16* p_Value);

uint8_t GetSignF16(Float16* p_Value);
int8_t GetExponentF16(Float16* p_Value);

void NegateF16(Float16* p_Value);

Float16 mul16(Float16* p_first, Float16* p_second);

Float16 add16(Float16* p_first, Float16* p_second);

Float16 div16(Float16* p_first, Float16* p_second);

Float16 sumUI16(const uint16_t* p_data, int p_n);

Float16 sumSI16(const int16_t* p_data, int p_n);

Float16 sqrt16(Float16* p_Arg);

Float16 log2_16(Float16* p_Arg);

Float16 log10_16(Float16* p_Arg);

void MomentsSI16(int16_t p_mean, const int16_t* p_data, int p_n, Float16* p_moments, int8_t p_maxMoment);

void MomentsUI16(uint16_t p_mean, const uint16_t* p_data, int p_n, Float16* p_moments, int8_t p_maxMoment);



Float16 sumUI8(const uint8_t* p_data, int p_n);

Float16 sumSI8(const int8_t* p_data, int p_n);

void MomentsSI8(int8_t p_mean, const int8_t* p_data, int p_n, Float16* p_moments, int8_t p_maxMoment);

void MomentsUI8(uint8_t p_mean, const uint8_t* p_data, int p_n, Float16* p_moments, int8_t p_maxMoment);

Float16 add16NA(uint16_t p_firstMantissa, int8_t p_firstExp, uint8_t p_firstSign, uint16_t p_secondMantissa, int8_t p_secondExp, uint8_t p_secondSign);

void MulAndAccumulate(int32_t p_accumulator,int16_t p_tmp, uint8_t p_firstArg, int8_t p_secondArg);

#ifdef __cplusplus
}
#endif

#endif // FLOAT16_H
