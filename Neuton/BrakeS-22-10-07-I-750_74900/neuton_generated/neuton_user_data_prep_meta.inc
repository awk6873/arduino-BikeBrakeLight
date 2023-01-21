/* 2022-12-08T05:33:50Z */

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
static const neuton_input_t INPUT_FEATURES_SCALE_MIN[] = { 0 };

/** Defines input(also used for LAG) features MAX scaling factor
 */
static const neuton_input_t INPUT_FEATURES_SCALE_MAX[] = { 0 };

/** Defines which unique features from the input data will be used/collected,
 *  one bit for one unique feature, starting from LSB
 */
static const neuton_u8_t INPUT_FEATURES_USAGE_MASK[] = { 0x6b, 0x01 };

/** Defines which unique input features is used for LAG features processing,
 *  one bit for one unique feature, starting from LSB
 */
static const neuton_u8_t INPUT_FEATURES_USED_FOR_LAGS_MASK[] = { 0x00, 0x00 };

/** Defines DSP feature calculation masks used as neuton_dsp_features_mask_t,
 *  32 bit for one unique feature, @ref neuton_dsp_features_mask_t to see bitmask
 */
static const neuton_u32_t DSP_FEATURES_CALCULATION_MASK[] = {
	17301504, 70778880, 11599872, 68157440, 524288, 65536 };

/** Defines DSP features MAX scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MIN[] = {
	-340, 44, -350, 2, -24129, 0, -3125, 1, -25036, 3, 4, 0, -2988, -14872 };

/** Defines DSP features MIN scaling factor
 */
static const neuton_feature_t DSP_FEATURES_SCALE_MAX[] = {
	531, 587, 366, 1111, 18084, 87, 5, 344, 27050, 562, 1015, 71, 1887, 4957 };

