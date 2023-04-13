/* 2023-03-25T15:44:45Z */

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
	-65536, -59244544, 1936850944, -9306112, 1870069760, 175702016, -940441600,
	-560660480, -1882259456 };

/** Defines DSP feature calculation arguments used by DSP pipeline,
 *  32 bit for one feature argument
 */
static const neuton_input_t DSP_FEATURES_CALCULATION_ARGS[] = {
	0, 4, 4, 0, 4, 4, 4, 4, 0, 4, 4, 0, 4, 0, 0, 4, 0 };

/** Defines DSP features MAX scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MIN[] = {
	-2878, -211, 13, -363, 2, -25498, 5352, 3, 42, 1, 0, 0, 5, 9, 33, 2, -389,
	2, -27288, 5600, 0, 0, 5, 10, 4, 3, -988, 4, -29469, 6095, 747, 1, 7, 16,
	731, -13, 1, -26149, 6269, 3, 1, 0, 0, 4, 8, 1, 2, -3819, -16, 12, 2, -27908,
	7118, 3, 1, 0, 0, 9, 2, -3282, -582, 3, -29460, 7090, 1, 0, -2735, 2, -14984,
	4756, 3, 5, 1, 0, 4, 2, -10936, 9, 2, 3, 1, 0, 0, 4, 9, 2, -14872, -4674,
	10, -8688, 4174, 3, 14, 1, 0, 0, 2 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	452, 4012, 4374, 542, 353, 26971, 195748, 580, 606, 76, 64, 64, 2010, 4269,
	542, 468, 383, 1138, 23392, 198376, 87, 87, 3216, 6865, 1150, 1507, 2885,
	1025, 25382, 219240, 1630, 73, 2769, 6572, 1255, 3842, 344, 27105, 197052,
	570, 76, 73, 73, 1682, 4244, 357, 459, -3, 4313, 8029, 1119, 23918, 204408,
	1514, 91, 89, 89, 6826, 1126, -16, 172, 1027, 25423, 219152, 71, 71, 6790,
	2914, 21589, 133624, 3358, 3517, 62, 53, 3270, 1485, 2527, 14351, 3876,
	4228, 64, 41, 41, 11744, 3898, 736, 5244, 12823, 25546, 7488, 185672, 9236,
	9253, 64, 37, 37, 1240 };

