/* 2023-04-21T06:56:48Z */

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
	31680, 26295, -12276, -15952, -11161, 11634, -4286, 22382, -30108, 32742,
	-21712, 18148, 12722, 2037, -14511, -18996, 22730, 24719, -14267, -18503,
	-9703, 28497, 8722, -17112, -32740, 1208, -5903, 32737, 17347, -20143,
	-20753, -11663, -12552, -20686, 31620, -32519, -1771, -12181, 31428, -30996,
	-13321, 2822, 20850, 32749, -32502, 25824, -20497, -27319, -32398, 7925,
	24573, -16388, -32768, -8136, -32768, -3, 6305, -28787, 32746, -32768,
	30867, -31544, -6, -27944, 25383, -32768, -32768, -8429, 8694, 6054, -32740,
	8046, 27418, -32720, 17542, 32712, 2171 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	3, 8, 14, 24, 26, 33, 36, 0, 0, 3, 6, 16, 30, 36, 1, 2, 4, 20, 21, 35,
	36, 2, 9, 12, 13, 28, 36, 3, 6, 11, 21, 33, 36, 1, 3, 4, 36, 0, 4, 0, 19,
	32, 36, 0, 1, 2, 3, 7, 31, 36, 1, 17, 25, 27, 34, 36, 6, 7, 14, 18, 22,
	29, 36, 5, 10, 15, 20, 23, 36, 0, 2, 6, 7, 8, 9, 10, 36 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 8, 15, 22, 27, 36, 39, 47, 50, 58, 63, 76 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	7, 14, 21, 27, 33, 37, 43, 50, 56, 63, 69, 77 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	40826, 20183, 14181, 27915, 40950, 10302, 30707, 40883, 40943, 40863, 40957,
	28160 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 11, 5 };

