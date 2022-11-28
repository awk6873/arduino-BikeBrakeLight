/* 2022-11-22T10:48:15Z */

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
	-0.29198512, -0.19478519, 0.81810957, -0.23129022, 0.99999881, 0.060460448,
	0.31872123, 0.38023156, 0.97820508, -0.12852509, 0.37255755, -0.30075541,
	-0.4240253, 0.47046718, -0.82168865, -0.34397855, -0.51311755, 0.51461482,
	-0.13022563, -0.78675187, 0.54238725, -0.99866933, -0.88335609, 0.99999994,
	-0.33594704, 0.37922668, -0.062272213, 0.99999094, 0.34441817, -0.99998927,
	-0.90954566, 0.85148025, 0.29364014, 0.90625, -0.58248365, 0.53799897,
	-0.017960727, -0.23981667, 0.013242271, -0.98437333, 0.36461863, 0.99829924,
	-0.86435884, 0.038012266, -0.99932444, -0.25645772, 0.55168045, -0.90791285,
	0.88121229, -0.90367401, 0.26461315, -1, 1, -0.80879104, 0.30162814, -0.023382187 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	58, 61, 96, 453, 486, 528, 0, 132, 307, 460, 484, 502, 528, 0, 1, 83, 301,
	453, 528, 0, 2, 3, 9, 37, 500, 528, 36, 193, 335, 386, 453, 511, 528, 0,
	2, 4, 528, 0, 2, 3, 4, 3, 453, 528, 93, 132, 163, 355, 384, 516, 528, 1,
	3, 6, 7, 528 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 7, 15, 21, 26, 36, 41, 44, 55 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	6, 13, 19, 26, 33, 37, 44, 51, 56 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	30.024975, 26.932156, 39.999985, 39.999981, 39.999962, 20.773388, 39.999985,
	39.999977, 40 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 5, 8 };

