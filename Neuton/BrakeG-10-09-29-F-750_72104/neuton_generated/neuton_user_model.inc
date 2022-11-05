/* 2022-11-05T12:33:21Z */

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
	-0.99371302, 0.73469681, 1, 0.53445798, 0.11468156, 0.0562675, 0.87553525,
	0, 1, 0.99414062, -0.16793445, 0.24971092, 0.22294492, -0.21875, -1, 0 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	0, 1, 2, 3, 6, 7, 0, 7, 0, 2, 3, 4, 5, 7, 2, 7 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 7, 9, 15 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	6, 8, 14, 16 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = { 39.999992, 40, 40, 40 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 1, 3 };

