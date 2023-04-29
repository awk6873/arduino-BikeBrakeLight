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


static const neuton_weight_t MODEL_WEIGHTS[] = {
	-11489, -9056, 28031, -14714, 15006, 8139, 13178, -11121, -11641, 15723,
	-19484, 24782, 0, -30500, 23988, -20960, -32744, 14758, 16666, 6003, 13357,
	29727, 11248, -7928, -16999, 21244, -31995, -17068, 24070, 1071, 16841,
	18733, 2038, 15570, -3212, -7911, -18779, -29226, 32756, -19649, 32752,
	-15676, -31600, -1031, 12073, -30686, -24448, 22609, 11493 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	2, 363, 376, 451, 471, 477, 0, 11, 154, 286, 363, 371, 477, 1, 367, 376,
	392, 461, 472, 477, 2, 45, 141, 251, 376, 407, 477, 1, 3, 477, 3, 68, 111,
	160, 180, 353, 477, 18, 27, 42, 57, 277, 471, 477, 0, 2, 5, 6, 477 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 7, 14, 21, 29, 31, 37, 48 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	6, 13, 20, 27, 30, 37, 44, 49 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	38077, 27128, 8399, 21997, 20203, 30744, 40932, 14145 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 7, 4 };

