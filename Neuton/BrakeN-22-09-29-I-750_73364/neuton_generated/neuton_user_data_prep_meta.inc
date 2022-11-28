/* 2022-11-22T10:12:57Z */

/* ----------------------------------------------------------------------
Copyright (c) 2021-2022 Bell Integrator, Inc.
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
* source code or its binary form without specific prior written permission of Bell Integrator, Inc.
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
	-29480, -11610, -17840, -2195, -2288, -4943 };

/** Defines input(also used for LAG) features MAX scaling factor
 */
static const neuton_input_t INPUT_FEATURES_SCALE_MAX[] = {
	32668, 16578, 32467, 1904, 4133, 3752 };

/** Defines which unique input features is used for LAG features processing,
 *  one bit for one unique feature, starting from LSB
 */
static const neuton_u8_t INPUT_FEATURES_USED_FOR_LAGS_MASK[] = { 0x3f };

/** Defines DSP feature calculation masks used as neuton_dsp_features_mask_t,
 *  32 bit for one unique feature, @ref neuton_dsp_features_mask_t to see bitmask
 */
static const neuton_u32_t DSP_FEATURES_CALCULATION_MASK[] = { 939458560 };

/** Defines DSP feature calculation arguments used by DSP pipeline,
 *  32 bit for one feature argument
 */
static const neuton_input_t DSP_FEATURES_CALCULATION_ARGS[] = {
	32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32 };

/** Defines DSP features MAX scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MIN[] = {
	-29480, -2548, 116, -3145, 24, -17642, 6726, 30, 178, 1, 0, 6, 112, -11610,
	-2374, 128, -3264, 24, -16415, 6326, 30, 603, 1, 0, 5, 107, -17840, 7988,
	196, 6812, 34, -17727, 7586, 43, 7483, 14, 0, 8, 171, -2195, -636, 5, -788,
	0, -7983, 4739, 1, 25, 1, 0, 0, 3, -2288, -269, 5, -471, 0, -10152, 5433,
	1, 20, 1, 0, 0, 3, -4943, -1275, 4, -1958, 0, -8524, 4984, 1, 8, 1, 0,
	0, 3 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	1116, 32668, 59544, 1188, 6772, 11463, 125208, 10041, 10055, 71, 67, 3736,
	58463, 1706, 16578, 23672, 1860, 2543, 25029, 195208, 3714, 3744, 70, 66,
	5258, 23970, 7968, 32467, 44880, 9303, 7155, 27303, 219816, 9119, 12509,
	66, 16, 5263, 45004, 468, 1904, 3147, 864, 1040, 10594, 41634, 1094, 1193,
	55, 22, 1688, 1515, 431, 4133, 5831, 710, 859, 13755, 56512, 1234, 1267,
	58, 32, 1200, 5849, 1592, 3752, 7548, 2180, 2508, 8657, 43964, 2712, 2936,
	51, 18, 5579, 3447 };

