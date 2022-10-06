/*
  @licstart 
  https://neuton.ai/license
  @licend 
*/

#include "float16.h"

#include <stdlib.h>

static const uint8_t highBitHelper[] = { 0,1,2,2,3,3,3,3,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8 };

// Inverse log base 2 of e
#define INV_LOG2_E_Q1DOT7  (uint16_t)(0x58)
#define INV_LOG2_E_Q1DOT15  (uint32_t)(0x58b9)
#define INV_LOG2_E_Q1DOT31  (uint64_t)(0x58b90bfc)

// Inverse log base 2 of 10
#define INV_LOG2_10_Q1DOT7 (uint16_t)(0x26)
#define INV_LOG2_10_Q1DOT15 (uint32_t)(0x2688)
#define INV_LOG2_10_Q1DOT31 (uint64_t)(0x268826a1)

void InitF16(Float16* p_Value, uint16_t p_Mantissa, int8_t p_Exponent, uint8_t p_Sign)
{
	p_Value->mantissa = p_Mantissa;
	if (p_Exponent < 0)
	{
		p_Value->extExponent = -p_Exponent;
		p_Value->extExponent |= 0x80 | p_Sign;
	}
	else
	{
		p_Value->extExponent = p_Exponent | p_Sign;
	}
}

uint32_t FixExpUF16(Float16* p_Value, int8_t p_Exponent)
{
	const int8_t exponent = GetExponentF16(p_Value);
	int8_t shift = p_Exponent - exponent;

	if (shift > 0)
	{
		return p_Value->mantissa >> shift;
	}
	shift = -shift;
	return p_Value->mantissa << shift;
}

int32_t FixExpF16(Float16* p_Value, int8_t p_Exponent)
{
	const int8_t exponent = GetExponentF16(p_Value);
	int8_t shift = p_Exponent - exponent;

	if (shift > 0)
	{
		if (GetSignF16(p_Value))
			return -(p_Value->mantissa >> shift);
		else
			return p_Value->mantissa >> shift;
	}
	shift = -shift;

	if (GetSignF16(p_Value))
		return -(p_Value->mantissa << shift);
	else
		return p_Value->mantissa << shift;
}

void AlignmentF16(Float16* p_Value)
{
	uint16_t data = p_Value->mantissa;

	int8_t shift = 8 - highBitHelper[data>>8];
	if (shift == 8)
		shift = 16 - highBitHelper[(uint8_t)data];

	p_Value->mantissa <<= shift;

	int8_t exp = GetExponentF16(p_Value);
	uint8_t sign = GetSignF16(p_Value);

	exp = exp - shift;

	if (exp < 0)
	{
		p_Value->extExponent = -exp;
		p_Value->extExponent |= 0x80 | sign;
	}
	else
	{
		p_Value->extExponent = exp | sign;
	}	
}

extern inline uint8_t GetSignF16(Float16* p_Value)
{
	return (p_Value->extExponent & 0x40);
}

extern inline int8_t GetExponentF16(Float16* p_Value)
{
	if (p_Value->extExponent & 0x80)
		return  -(p_Value->extExponent & 0x3F);
	else
		return  p_Value->extExponent & 0x3F;
}

void NegateF16(Float16* p_Value)
{
	p_Value->extExponent = (p_Value->extExponent ^ 0x40);
}

Float16 mul16(Float16* p_first, Float16* p_second)
{
	const int8_t sign = GetSignF16(p_first) ^ GetSignF16(p_second);
	Float16 result;
	const uint32_t mulRes = p_first->mantissa * p_second->mantissa;

	uint8_t shift = 8 + highBitHelper[mulRes>>24];
	if (shift == 8)
		shift = highBitHelper[(uint8_t)(mulRes>>16)];

	InitF16(&result, mulRes >> shift, (GetExponentF16(p_first) + GetExponentF16(p_second) + shift), sign);
	return result;
}

Float16 div16(Float16* p_first, Float16* p_second)
{
	Float16 result;

	uint8_t sign = GetSignF16(p_first) ^ GetSignF16(p_second);

	uint16_t data =p_first->mantissa;

	int8_t first_shift = 7 - highBitHelper[data>>8];
	if (first_shift == 8)
		first_shift = 15 - highBitHelper[(uint8_t)data];

	data = p_second->mantissa;

	int8_t second_shift = 8 + highBitHelper[data >> 8];
	if (second_shift == 8)
		second_shift = highBitHelper[(uint8_t)data];

	first_shift += second_shift;

	InitF16(&result, ((p_first->mantissa << first_shift) / p_second->mantissa), (GetExponentF16(p_first) - GetExponentF16(p_second) - first_shift), sign);

	return result;
}

Float16 add16NA(uint16_t p_firstMantissa, int8_t p_firstExp, uint8_t p_firstSign, uint16_t p_secondMantissa, int8_t p_secondExp, uint8_t p_secondSign)
{
	int8_t sign;

	Float16 result;

	int8_t shift = p_firstExp - p_secondExp;

	int32_t addRes;

	if (p_firstSign == p_secondSign)
	{
		if (shift > 0)
		{
			if (shift > 15)
			{
				InitF16(&result, p_firstMantissa, p_firstExp, p_firstSign);
				return result;
			}

			addRes = (p_secondMantissa >> shift) + p_firstMantissa;

			if (!(addRes & 0xFFFF0000))
			{
				InitF16(&result, addRes, p_firstExp, p_firstSign);
				return result;
			}
			InitF16(&result, addRes >> 1, p_firstExp + 1, p_firstSign);

			return result;
		}
		else
		{
			if (shift < -15)
			{
				InitF16(&result, p_secondMantissa, p_secondExp, p_secondSign);
				return result;
			}

			shift = -shift;

			addRes = (p_firstMantissa >> shift) + p_secondMantissa;

			if (!(addRes & 0xFFFF0000))
			{
				InitF16(&result, addRes, p_secondExp, p_firstSign);
				return result;
			}
			InitF16(&result, addRes >> 1, p_secondExp + 1, p_firstSign);

			return result;
		}
	}
	else
	{
		if (shift >= 0)
		{
			if (shift > 15)
			{
				InitF16(&result, p_firstMantissa, p_firstExp, p_firstSign);
				return result;
			}

			addRes = p_firstMantissa - (p_secondMantissa >> shift);

			if (addRes > 0)
				sign = p_firstSign;
			else if (addRes < 0)
			{
				sign = p_secondSign;
				addRes = -addRes;
			}
			else sign = 0;

			InitF16(&result, addRes, p_firstExp, sign);
			return result;
		}
		else
		{
			if (shift < -15)
			{
				InitF16(&result, p_secondMantissa, p_secondExp, p_secondSign);
				return result;
			}

			addRes = (p_firstMantissa >> (-shift)) - p_secondMantissa;

			if (addRes > 0)
				sign = p_firstSign;
			else if (addRes < 0)
			{
				sign = p_secondSign;
				addRes = -addRes;
			}
			else sign = 0;

			Float16 result;

			InitF16(&result, addRes, p_secondExp, sign);
			return result;
		}
	}
}


Float16 add16(Float16* p_first, Float16* p_second)
{
	uint16_t fmantissa = p_first->mantissa;

	int8_t shift = 8 - highBitHelper[fmantissa >> 8];
	if (shift == 8)
		shift = 16 - highBitHelper[(uint8_t)fmantissa];

	fmantissa <<= shift;

	int8_t firstExp = GetExponentF16(p_first);
	const uint8_t firstSign = GetSignF16(p_first);

	firstExp -= shift;

	uint16_t smantissa = p_second->mantissa;

	shift = 8 - highBitHelper[smantissa >> 8];
	if (shift == 8)
		shift = 16 - highBitHelper[(uint8_t)smantissa];

	smantissa <<= shift;

	int8_t secondExp = GetExponentF16(p_second);
	const uint8_t secondSign = GetSignF16(p_second);

	secondExp -= shift;

	return add16NA(fmantissa, firstExp, firstSign, smantissa, secondExp, secondSign);
}


Float16 sqrt16(Float16* p_Arg)
{
	Float16 resultF;

	uint8_t shift = 16U;

	int8_t exp = GetExponentF16(p_Arg);

	if (exp % 2)
		shift = 15;

	exp = exp >> 1;

	uint32_t arg = p_Arg->mantissa << shift;

	shift = shift >> 1;

	exp = exp - shift;

	if (arg == 0) {
		InitF16(&resultF, 0, 0, 0);
		return resultF;
	}

	uint32_t div;
	uint32_t result = arg;

	if (arg & 0xFFFF0000L)
		if (arg & 0xFF000000L)
			div = 0x3FFF;
		else
			div = 0x3FF;
	else
		if (arg & 0x0FF00L) div = 0x3F;
		else div = (arg > 4) ? 0x7 : arg;

	while (1)
	{
		const uint32_t temp = arg / div + div;
		div = temp >> 1;
		div += temp & 1;
		if (result > div) result = div;
		else
		{
			if (1 / result == result - 1 && 1 % result == 0) result--;

			InitF16(&resultF, result, exp, 0);

			return resultF;
		}
	}
}

Float16 log2_16(Float16* p_Arg)
{
	Float16 result;

	uint16_t data = p_Arg->mantissa;

	int8_t shift = 8 + highBitHelper[data>>8];
	if (shift == 8)
		shift = highBitHelper[(uint8_t)data];

	if (shift < 15)
		shift = 15 - shift;
	else
		shift = 0;

	uint16_t x = p_Arg->mantissa << shift;

	int8_t exponent = GetExponentF16(p_Arg) - shift;

	if(abs(exponent)>15)
	{
		x = x >> (abs(exponent)-15);
		exponent = -15;
	}

	const int8_t precision = abs(exponent);

	if (precision < 1 || precision > 15)
	{
		InitF16(&result, INT16_MAX, 0, 0);
		return result; // indicates an error
	}

	if (x == 0)
	{
		InitF16(&result, INT16_MIN, 0, 0);
		return result; // represents negative infinity
	}


	int16_t b = (uint16_t)1U << (precision - 1);
	int16_t y = 0;

	while (x < (uint16_t)1U << precision)
	{
		x <<= 1;
		y -= (uint16_t)1U << precision;
	}

	while (x >= (uint32_t)2U << precision)
	{
		x >>= 1;
		y += (uint16_t)1U << precision;
	}

	uint32_t z = x;

	for (uint8_t i = 0; i < precision; ++i)
	{
		z = z * z >> precision;
		if (z >= (uint32_t)2U << precision)
		{
			z >>= 1;
			y += b;
		}
		b >>= 1;
	}

	if(y<0)
		InitF16(&result, ~y + 1, exponent, 0x40);
	else
		InitF16(&result, y, exponent, 0);

	return result;
}

Float16 log10_16(Float16* p_Arg)
{
	Float16 result = log2_16(p_Arg);

	int32_t t;
	t = result.mantissa * INV_LOG2_10_Q1DOT15;

	result.mantissa = (int16_t)(t >> 15);

	return result;
}

Float16 sumUI16(const uint16_t* p_data, int p_n)
{
	int8_t resultExp = 0;

	uint32_t addRes = 0;

	uint8_t* data = (uint8_t*)&addRes;
	data = data + 3;

	for (int i = 0; i < p_n; i++)
	{
		addRes += (*p_data) >> resultExp;

		if(*data)
		{
			uint8_t shift = highBitHelper[(uint8_t)(addRes >> 16)];

			addRes >>= shift;

			resultExp += shift;

		}
		p_data++;
	}

	uint8_t shift = highBitHelper[(uint8_t)(addRes >> 16)];

	addRes >>= shift;

	resultExp += shift;

	Float16 result;
	InitF16(&result, addRes, resultExp, 0);

	return result;
}


Float16 sumSI16(const int16_t* p_data, int p_n)
{
	int8_t resultExpP = 0;
	int8_t resultExpM = 0;

	uint32_t addResP = 0;
	uint32_t addResM = 0;

	uint8_t* dataP = (uint8_t*)&addResP;
	dataP = dataP + 3;
	uint8_t* dataM = (uint8_t*)&addResM;
	dataM = dataM + 3;

	for (int i = p_n; i > 0; i--)
	{
		int16_t value = (*p_data);

		if(value>0)
		{
			addResP += value >> resultExpP;

			if (*dataP)
			{
				uint8_t shift = highBitHelper[(uint8_t)(addResP >> 16)];

				addResP >>= shift;

				resultExpP += shift;
			}
		} else
		{
			addResM += (-value) >> resultExpM;

			if (*dataM)
			{
				uint8_t shift = highBitHelper[(uint8_t)(addResM >> 16)];

				addResM >>= shift;

				resultExpM += shift;
			}
		}
		p_data++;
	}

	uint8_t shift = highBitHelper[(uint8_t)(addResP >> 16)];

	addResP >>= shift;

	resultExpP += shift;
	shift = highBitHelper[(uint8_t)(addResM >> 16)];

	addResM >>= shift;

	resultExpM += shift;

	return add16NA(addResP, resultExpP, 0, addResM, resultExpM, 64);
}

void MomentsSI16(int16_t p_mean, const int16_t* p_data, int p_n, Float16* p_moments, int8_t p_maxMoment)
{
	for (int i = p_n; i > 0; i--)
	{
		int16_t value = (*p_data);

		int32_t diff = value - p_mean;

		uint8_t sign = diff >= 0 ? 0 : 64;

		value = abs(diff);

		int8_t shift = (int8_t)8 + highBitHelper[value >> 8];
		if (shift == (int8_t)8)
			shift = highBitHelper[(uint8_t)value];

		const int8_t hBitMul = (int8_t)16 - shift;

		uint32_t mulRes = value << hBitMul;

		int8_t resultExp = -hBitMul;

		Float16* moments = p_moments;

		for(int8_t p_moment = p_maxMoment; p_moment>0; p_moment--)
		{
			mulRes *= value;

			mulRes >>= shift;

			resultExp += shift;

			if(p_moment&(int8_t)1)
				*moments = add16NA(mulRes, resultExp, 0, moments->mantissa, GetExponentF16(moments), GetSignF16(moments));
			else
				*moments = add16NA(mulRes, resultExp, sign, moments->mantissa, GetExponentF16(moments), GetSignF16(moments));

			moments++;
		}
		p_data++;
	}
	Float16* moments = p_moments;
	for (int8_t p_moment = p_maxMoment; p_moment > 0; p_moment--)
	{
		uint16_t data = moments->mantissa;

		int8_t first_shift = (int8_t)7 - highBitHelper[data >> 8];
		if (first_shift == (int8_t)8)
			first_shift = (int8_t)15 - highBitHelper[(uint8_t)data];

		data = p_n;

		int8_t second_shift = (int8_t)8 + highBitHelper[data >> 8];
		if (second_shift == (int8_t)8)
			second_shift = highBitHelper[(uint8_t)data];

		first_shift += second_shift;

		InitF16(moments, ((moments->mantissa << first_shift) / data), (GetExponentF16(moments) - first_shift), GetSignF16(moments));

		moments++;
	}
}

void MomentsUI16(uint16_t p_mean, const uint16_t* p_data, int p_n, Float16* p_moments, int8_t p_maxMoment)
{
	for (int i = p_n; i > 0; i--)
	{
		uint16_t value = (*p_data);

		int32_t diff = value - p_mean;

		uint8_t sign = diff >= 0 ? 0 : 64;

		value = abs(diff);

		int8_t shift = (int8_t)8 + highBitHelper[value >> 8];
		if (shift == (int8_t)8)
			shift = highBitHelper[(uint8_t)value];

		const int8_t hBitMul = (int8_t)16 - shift;

		uint32_t mulRes = value << hBitMul;

		int8_t resultExp = -hBitMul;

		Float16* moments = p_moments;

		for (int8_t p_moment = p_maxMoment; p_moment > 0; p_moment--)
		{
			mulRes *= value;

			mulRes >>= shift;

			resultExp += shift;

			if (p_moment&(int8_t)1)
				*moments = add16NA(mulRes, resultExp, 0, moments->mantissa, GetExponentF16(moments), GetSignF16(moments));
			else
				*moments = add16NA(mulRes, resultExp, sign, moments->mantissa, GetExponentF16(moments), GetSignF16(moments));

			moments++;
		}
		p_data++;
	}
	Float16* moments = p_moments;
	for (int8_t p_moment = p_maxMoment; p_moment > 0; p_moment--)
	{
		uint16_t data = moments->mantissa;

		int8_t first_shift = (int8_t)7 - highBitHelper[data >> 8];
		if (first_shift == (int8_t)8)
			first_shift = (int8_t)15 - highBitHelper[(uint8_t)data];

		data = p_n;

		int8_t second_shift = (int8_t)8 + highBitHelper[data >> 8];
		if (second_shift == (int8_t)8)
			second_shift = highBitHelper[(uint8_t)data];

		first_shift += second_shift;

		InitF16(moments, ((moments->mantissa << first_shift) / data), (GetExponentF16(moments) - first_shift), GetSignF16(moments));

		moments++;
	}
}
