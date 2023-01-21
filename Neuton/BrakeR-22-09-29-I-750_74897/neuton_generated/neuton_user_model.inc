/* 2022-12-07T16:23:12Z */

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
	-28191, 25608, 32764, 15425, 13665, -2782, 32764, 11626, -3271, 4581, -8488,
	-7791, -32730, 1467, -32766, 9516, -7562, -775, -4928, -9, 32767, -17405,
	-2 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	0, 2, 5, 6, 9, 13, 0, 4, 7, 8, 10, 13, 1, 13, 1, 1, 3, 11, 12, 13, 0, 3,
	13 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 7, 13, 15, 22 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	6, 12, 14, 20, 23 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	40960, 40930, 40958, 40956, 40958 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 4, 2 };

