/* 2022-11-05T11:01:15Z */

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


static const neuton_weight_t MODEL_WEIGHTS[] = {
	-32768, 28669, 32765, -2270, -1687, -2, 32451, -4469, -13516, 7521, -32768,
	22532, -6325, -29047, 8679, -16007, 19994, 23230, 29781, -32768, -6570,
	-9294, 26536, -17009, -1327 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	0, 1, 2, 11, 13, 15, 0, 3, 4, 5, 9, 10, 15, 1, 15, 1, 6, 7, 8, 12, 14,
	15, 0, 3, 15 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 7, 14, 16, 24 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	6, 13, 15, 22, 25 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	40954, 29727, 13889, 40950, 12877 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 4, 2 };

