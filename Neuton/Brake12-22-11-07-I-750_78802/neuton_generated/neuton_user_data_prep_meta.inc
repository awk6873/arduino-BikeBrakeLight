/* 2023-04-16T22:27:58Z */

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
	-149028864, -141688832, -140574720, -140836864, -142016512, -1759969280,
	-140574720, -719388672, -140836864 };

/** Defines DSP feature calculation arguments used by DSP pipeline,
 *  32 bit for one feature argument
 */
static const neuton_input_t DSP_FEATURES_CALCULATION_ARGS[] = {
	4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 };

/** Defines DSP features MAX scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MIN[] = {
	-211, 12, -363, 2, 42, 1, 0, 5, 8, 33, 2, -340, 12, -390, 3, 6, 1, 0, 5,
	9, 4, 3, -3988, -988, 21, -1138, 3, 138, 747, 1, 0, 7, 16, 731, 5, -3125,
	-13, -278, 1, 2, 2, 1, 0, 4, 8, 1, 2, -3819, -611, 3, 3, 1, 0, 4, 9, 2,
	2, -3282, -582, 3, 7, 1, 0, 6, 5, -9564, -2784, 9, -3271, 2, 3, 5, 1, 0,
	3, 8, 4, 2, -10936, -1616, 9, -3662, 1, 12, 0, 4, 9, 2, -14872, -4674,
	-8688, 2, 3, 14, 1, 0, 4, 8, 11, 2 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	4012, 4374, 544, 354, 609, 78, 64, 2010, 4269, 544, 468, 3975, 7795, 384,
	1531, 1533, 89, 87, 3216, 6865, 1151, 1518, -919, 2885, 6873, -731, 1025,
	1794, 1636, 73, 33, 2769, 6572, 1262, 1241, 11, 3842, 342, 345, 577, 577,
	76, 75, 1682, 4244, 358, 459, -3, 698, 1523, 1524, 91, 89, 3244, 6826,
	1133, 1500, -15, 175, 1027, 1384, 71, 71, 2935, 1242, 1415, 6790, 14086,
	2129, 2914, 3550, 3517, 62, 53, 11106, 7226, 3271, 1487, 2750, 7101, 14351,
	3863, 3893, 4247, 42, 11744, 3898, 736, 5244, 12823, 7500, 8501, 9265,
	9266, 64, 37, 25075, 10802, 8688, 1240 };

