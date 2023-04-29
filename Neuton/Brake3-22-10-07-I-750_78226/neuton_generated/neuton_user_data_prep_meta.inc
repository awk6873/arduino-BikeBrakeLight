/* 2023-03-19T11:11:10Z */

/* ----------------------------------------------------------------------
Copyright (c) 2022-2023 Neuton.AI, Inc.
*
The source code and its binary form are being made available on the following terms:
Redistribution, use, and modification are permitted provided that the following
conditions are met:
*
1. Redistributions of source code and/or its binary form must retain the above copyright notice,
* this list of conditions (and the disclaimer) either in the body of the source code or in
* the documentation and/or other materials provided with the distribution of the binary form, as
applicable.
*
2. The name of the copyright holder may not be used to endorse or promote products derived from this
* source code or its binary form without specific prior written permission of Neuton.AI, Inc.
*
3. Disclaimer. THIS SOURCE CODE AND ITS BINARY FORM ARE PROVIDED BY THE COPYRIGHT HOLDER "AS IS".
* THE COPYRIGHT HOLDER HEREBY DISCLAIMS ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
* BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE HELD LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS
* OF THIRD PARTIES; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOURCE CODE OR ITS BINARY FORM, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------------- */

/** Defines input(also used for LAG) features MIN scaling factor
 */
static const neuton_input_t INPUT_FEATURES_SCALE_MIN[] = {
	-2878, -3820, -3988, -3125, -3819, -3282, -9564, -10936, -14872 };

/** Defines input(also used for LAG) features MAX scaling factor
 */
static const neuton_input_t INPUT_FEATURES_SCALE_MAX[] = {
	4012, 3975, 2885, 3842, 4313, 3874, 6790, 7101, 12823 };

/** Defines which unique input features is used for LAG features processing,
 *  one bit for one unique feature, starting from LSB
 */
static const neuton_u8_t INPUT_FEATURES_USED_FOR_LAGS_MASK[] = { 0xff, 0x01 };

/** Defines DSP feature calculation masks used as neuton_dsp_features_mask_t,
 *  32 bit for one unique feature, @ref neuton_dsp_features_mask_t to see bitmask
 */
static const neuton_u32_t DSP_FEATURES_CALCULATION_MASK[] = { -140574720 };

/** Defines DSP feature calculation arguments used by DSP pipeline,
 *  32 bit for one feature argument
 */
static const neuton_input_t DSP_FEATURES_CALCULATION_ARGS[] = {
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 };

/** Defines DSP features MAX scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MIN[] = {
	-2878, -211, 13, -340, 2, 3, 44, 1, 0, 5, 11, 35, 310, -3820, -338, 14,
	-350, 2, 4, 6, 1, 0, 6, 12, 5, 364, -3988, -986, 22, -1103, 4, 1150, 869,
	3, 0, 8, 18, 842, 594, -3125, -9, 12, -248, 1, 3, 3, 1, 0, 5, 10, 1, 283,
	-3819, 3, 12, -549, 2, 3, 3, 3, 3, 5, 11, 2, 319, -3282, -2, 22, -521,
	4, 5, 8, 3, 0, 8, 18, 5, 582, -9564, -2146, 9, -2988, 2, 3, 6, 1, 0, 5,
	9, 4, 278, -10936, -1314, 12, -2690, 2, 3, 12, 1, 0, 5, 9, 10, 246, -14872,
	-4674, 13, -7306, 2, 3, 18, 1, 0, 5, 10, 13, 275 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	452, 4012, 4374, 531, 351, 573, 587, 71, 58, 2010, 4269, 531, 46876, 184,
	3975, 7560, 366, 1111, 1505, 1506, 87, 87, 3216, 6865, 1111, 150716, -923,
	2885, 6873, -833, 1014, 1785, 1607, 67, 25, 2769, 6572, 1215, 124157, 5,
	3842, 4327, 294, 344, 563, 563, 73, 69, 1682, 4244, 348, 45996, -5, 4313,
	7681, 624, 1077, 1491, 1491, 87, 87, 3244, 6826, 1076, 148767, -16, 3874,
	6873, 138, 1015, 1366, 1373, 67, 71, 2769, 6573, 1041, 124237, 375, 6790,
	14086, 1887, 2706, 3425, 3368, 60, 44, 11106, 7226, 2988, 135382, 1701,
	7101, 12845, 3054, 3308, 3816, 3774, 60, 39, 10668, 5171, 3294, 58860,
	4957, 12823, 23695, 6268, 7945, 8920, 8887, 60, 33, 21912, 10802, 7781,
	97955 };

