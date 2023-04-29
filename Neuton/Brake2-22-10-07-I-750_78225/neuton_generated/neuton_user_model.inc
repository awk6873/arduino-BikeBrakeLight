/* 2023-03-19T09:51:16Z */

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


static const neuton_weight_t MODEL_WEIGHTS[] = {
	-32768, 27977, -11144, -2869, 14575, 13506, 5695, 28113, 24201, 27068,
	-23308, -16607, -6013, -15601, 4231, -27134, 11603, 20736, 14340, -12694,
	-2763, -13126, 4321, 6225, -8900, 32764, -12589, -21852, -32768, 19877,
	32692, -31219, 25650, -8, 7924, -767, 21095, 13809, -32768, -17373, -32768,
	-16389, 25089, -32768, 24372, -4, 31461, 29442, -28979, -19339, 23513,
	-29670, 5218, -22665, 32765, 30004, 26681, 9708, 17610, -954 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	0, 4, 6, 7, 10, 25, 27, 0, 2, 5, 9, 14, 27, 1, 27, 1, 19, 20, 25, 26, 27,
	0, 1, 3, 0, 15, 16, 27, 1, 11, 13, 17, 26, 27, 0, 3, 9, 14, 25, 27, 11,
	12, 17, 22, 24, 27, 8, 10, 18, 21, 23, 27, 0, 3, 4, 5, 6, 7, 8, 27 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 8, 14, 16, 24, 28, 35, 40, 46, 59 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	7, 13, 15, 21, 28, 34, 40, 46, 52, 60 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	40958, 17336, 23733, 10316, 40957, 40957, 37480, 40954, 35527, 31700 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 9, 2 };

