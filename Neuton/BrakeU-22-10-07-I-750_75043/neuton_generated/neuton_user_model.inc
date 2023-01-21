/* 2022-12-10T13:14:08Z */

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
	20303, -9900, -20686, 4922, -19789, 13493, 16949, 31815, -7514, 21921,
	-30038, -14326, 8225, -32768, -32768, -28673, 32765, -32768, -3, 32764,
	28391, -3804, -15371, -12529, -5440, -9799, 22598, -28836, 7051, 647, 1577,
	26240, 32764, -16963, 8090, -16123, 10797, -32686, 18686, -30681, -3796,
	-18861, -32734, 8769, 169 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	0, 4, 8, 15, 17, 23, 25, 0, 6, 7, 11, 21, 25, 5, 9, 10, 14, 19, 25, 2,
	2, 12, 13, 22, 25, 1, 2, 3, 15, 16, 25, 0, 2, 4, 25, 1, 3, 18, 20, 24,
	25, 1, 3, 6, 25 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 8, 13, 20, 28, 34, 35, 44 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	7, 13, 19, 25, 31, 35, 41, 45 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	6275, 24970, 40958, 30615, 30718, 14267, 40918, 13321 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 5, 7 };

