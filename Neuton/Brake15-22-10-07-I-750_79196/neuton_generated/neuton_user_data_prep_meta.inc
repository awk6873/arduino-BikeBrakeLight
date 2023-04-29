/* 2023-04-26T05:51:46Z */

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
	-2878, -210, 14, -343, 2, 3, 52, 1, 0, 5, 11, 41, 3, -3820, -339, 13, -350,
	2, 3, 6, 1, 0, 6, 10, 5, 3, -3988, -986, 22, -1117, 3, 335, 864, 1, 0,
	8, 18, 815, 5, -3125, -9, 13, -241, 2, 3, 3, 1, 0, 4, 10, 2, 3, -3819,
	3, 12, -548, 2, 3, 3, 3, 1, 5, 9, 2, 3, -3282, -2, 22, -431, 3, 5, 8, 1,
	0, 8, 18, 6, 5, -9564, -2689, 9, -3276, 2, 3, 6, 1, 0, 4, 8, 4, 2, -10936,
	-1085, 9, -3095, 2, 3, 14, 1, 0, 5, 9, 10, 2, -14872, -4448, 13, -7469,
	2, 3, 19, 1, 0, 5, 9, 14, 2 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	444, 4012, 4374, 532, 348, 578, 602, 72, 54, 2010, 4269, 532, 465, 235,
	3975, 7560, 354, 1120, 1527, 1530, 81, 81, 3216, 6865, 1134, 1512, -923,
	2885, 6873, -807, 995, 1798, 1597, 67, 25, 2769, 6572, 1211, 1227, 11,
	3842, 4327, 284, 342, 568, 568, 69, 72, 1682, 4244, 347, 457, -5, 4313,
	7681, 646, 1111, 1516, 1516, 85, 85, 3244, 6826, 1115, 1493, -17, 3874,
	6873, 156, 997, 1366, 1375, 67, 67, 2789, 6573, 1035, 1227, 375, 6790,
	14086, 1727, 2727, 3474, 3396, 60, 43, 11106, 6902, 3276, 1412, 1579, 7101,
	12845, 3837, 3136, 4220, 4179, 60, 32, 10193, 5171, 3872, 572, 5244, 12823,
	23695, 6183, 7878, 8948, 8978, 60, 34, 21912, 10802, 7723, 851 };

