/* 2023-04-27T05:13:10Z */

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
	-2878, -211, 13, -387, 2, 3, 55, 2, 0, 5, 8, 43, 2, -3820, -339, 14, -356,
	2, 3, 6, 2, 0, 5, 10, 4, 3, -3988, -988, 20, -1116, 3, 5, 767, 2, 0, 8,
	17, 753, 5, -3125, -7, 12, -286, 2, 3, 3, 2, 0, 4, 8, 2, 2, -3819, -16,
	12, -607, 2, 3, 3, 2, 0, 4, 9, 2, 3, -3282, -2, 21, -494, 3, 5, 7, 2, 0,
	7, 17, 5, 5, -9564, -2829, 9, -3322, 2, 3, 5, 2, 0, 5, 9, 4, 2, -10936,
	-1610, 9, -3780, 2, 3, 13, 2, 0, 4, 9, 10, 2, -14872, -4802, 13, -8228,
	2, 3, 14, 2, 0, 5, 9, 11, 2 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	462, 4012, 4374, 544, 401, 630, 645, 72, 61, 2010, 4269, 544, 476, 241,
	3975, 7402, 378, 1172, 1371, 1371, 85, 85, 3153, 6734, 1172, 1430, -919,
	2885, 6873, -753, 1015, 1744, 1643, 72, 27, 2769, 6572, 1241, 1239, 13,
	3842, 4327, 299, 390, 619, 619, 72, 70, 1682, 4244, 390, 464, 2, 4313,
	7402, 693, 1137, 1357, 1360, 89, 89, 3148, 6688, 1135, 1409, -16, 3874,
	6873, 188, 1017, 1436, 1449, 70, 70, 2789, 6573, 1063, 1240, 500, 6790,
	14086, 1988, 2755, 3368, 3347, 59, 46, 9142, 7226, 3322, 1471, 1323, 7101,
	12845, 3787, 3049, 4228, 4160, 65, 40, 10193, 5171, 3828, 634, 5244, 12823,
	23695, 7774, 6809, 9416, 9461, 61, 34, 21798, 10802, 8228, 1045 };

