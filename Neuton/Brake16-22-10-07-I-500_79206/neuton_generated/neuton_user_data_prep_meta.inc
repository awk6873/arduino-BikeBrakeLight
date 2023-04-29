/* 2023-04-26T10:58:29Z */

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
	-2878, -212, 13, -323, 2, 3, 43, 2, 0, 4, 9, 33, 2, -3820, -341, 11, -355,
	2, 3, 6, 2, 0, 3, 9, 4, 3, -3988, -988, 20, -1139, 3, 5, 816, 2, 0, 7,
	16, 769, 4, -3125, -25, 11, -328, 2, 3, 3, 2, 0, 4, 9, 2, 2, -3819, -16,
	11, -585, 2, 3, 3, 2, 0, 4, 9, 2, 3, -3282, -3, 19, -641, 3, 5, 7, 2, 0,
	6, 15, 5, 5, -9564, -2829, 9, -3318, 1, 2, 5, 2, 0, 3, 8, 3, 2, -10936,
	-2427, 9, -3450, 1, 3, 12, 2, 0, 3, 8, 8, 2, -14872, -6186, 7, -10054,
	2, 3, 13, 2, 0, 4, 7, 9, 2 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	447, 4012, 4374, 539, 402, 655, 689, 79, 69, 2010, 4269, 539, 493, 201,
	3975, 7560, 372, 1311, 1740, 1740, 94, 89, 3216, 6865, 1312, 1774, -919,
	2885, 6873, -769, 1191, 1557, 1740, 74, 30, 2769, 6572, 1301, 1384, 38,
	3842, 4327, 347, 392, 642, 642, 76, 76, 1682, 4244, 392, 482, 4, 4313,
	7681, 593, 1298, 1729, 1729, 89, 87, 3244, 6826, 1296, 1755, -15, 3874,
	6873, 210, 1192, 1558, 1572, 74, 69, 2813, 6573, 1238, 1385, 1122, 6790,
	14086, 2306, 3198, 3910, 3945, 66, 58, 11106, 7226, 3318, 1469, 2348, 7101,
	12845, 4408, 4228, 4612, 4681, 64, 38, 11507, 5171, 4408, 699, 5244, 12823,
	23695, 8402, 7682, 10312, 10290, 66, 33, 21798, 10802, 10054, 1022 };

