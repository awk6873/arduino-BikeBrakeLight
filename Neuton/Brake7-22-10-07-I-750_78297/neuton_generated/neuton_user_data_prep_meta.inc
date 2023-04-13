/* 2023-03-24T06:58:42Z */

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
static const neuton_input_t INPUT_FEATURES_SCALE_MIN[] = { 0 };

/** Defines input(also used for LAG) features MAX scaling factor
 */
static const neuton_input_t INPUT_FEATURES_SCALE_MAX[] = { 0 };

/** Defines which unique input features is used for LAG features processing,
 *  one bit for one unique feature, starting from LSB
 */
static const neuton_u8_t INPUT_FEATURES_USED_FOR_LAGS_MASK[] = { 0x00, 0x00 };

/** Defines DSP feature calculation masks used as neuton_dsp_features_mask_t,
 *  32 bit for one unique feature, @ref neuton_dsp_features_mask_t to see bitmask
 */
static const neuton_u32_t DSP_FEATURES_CALCULATION_MASK[] = {
	-174456832, -677511168, -1516371968, 2005860352, -1224605696, -158990336,
	-954531840, 1737687040, -2054225920 };

/** Defines DSP feature calculation arguments used by DSP pipeline,
 *  32 bit for one feature argument
 */
static const neuton_input_t DSP_FEATURES_CALCULATION_ARGS[] = {
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 };

/** Defines DSP features MAX scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MIN[] = {
	-210, -361, 2, 3, 43, 0, 5, 10, 33, 2, -340, 12, -384, 2, 3, 6, 1, 0, 5,
	4, 3, -988, 21, -1119, 4, 140, 747, 0, 17, 5, -3125, -13, 12, -278, 3,
	3, 1, 0, 4, 10, 1, -16, 3, 1, 0, 4, 9, 2, -2, 20, 5, 1, 0, 6, 16, 5, 5,
	-9564, -2454, -3262, 2, 5, 1, 0, 4, 2, -10936, -1552, 2, 3, 12, 1, 0, 9,
	9, -14872, -4674, 10, -8687, 3, 14, 0, 2 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	4012, 542, 353, 580, 606, 64, 2010, 4269, 542, 464, 3975, 7795, 380, 1137,
	1527, 1529, 89, 87, 3216, 1146, 1506, 2885, 6873, -731, 1024, 1794, 1627,
	33, 6572, 1239, 11, 3842, 4327, 332, 570, 570, 76, 73, 1682, 4244, 354,
	4313, 1514, 87, 87, 3244, 6826, 1486, 3874, 6873, 1379, 69, 71, 2935, 6573,
	1054, 1240, 1207, 6790, 2123, 2914, 3515, 62, 53, 3262, 1482, 2527, 7101,
	3893, 4233, 4246, 64, 41, 5171, 3898, 5244, 12823, 25546, 7500, 9238, 9253,
	35, 1225 };

