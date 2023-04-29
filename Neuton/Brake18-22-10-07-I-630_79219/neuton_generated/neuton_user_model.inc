/* 2023-04-27T05:13:10Z */

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
	-2798, -8528, -12740, 17553, 32764, -23863, 15106, 32765, -113, 5560, -29240,
	-13632, 11558, -11688, -5254, 21089, -21135, 14127, 148, 30898, -17617,
	-32763, 32765, -8556, -20916, -9596, -32728, 28644, 92, -31892, 32739,
	32709, -15084, 32761, -20515, -25560, -16338, -16257, -23130, -3124, 26125,
	22810, 12483, -9555, -9272, -20031, 32485, 32757, 30118, -14695, 11804,
	-27302, 32766, -32205, -6893, 588, -7161, -12555, 19634, 2991, -30956,
	-26437, 17301, 5062, 4169, -28179, 32761, -31819, -6085, 8494, -8627, 32764,
	-15829, 7862, -32768, 14734, 3340 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	40, 193, 312, 437, 448, 463, 549, 0, 19, 190, 435, 505, 549, 1, 482, 483,
	535, 546, 549, 2, 80, 82, 353, 525, 540, 549, 52, 76, 103, 131, 158, 281,
	549, 0, 59, 74, 200, 463, 543, 549, 5, 197, 382, 438, 491, 549, 185, 448,
	476, 497, 529, 549, 7, 52, 355, 373, 437, 467, 549, 0, 2, 5, 8, 549, 12,
	134, 192, 282, 439, 549, 1, 3, 4, 6, 7, 10, 549 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 8, 14, 20, 26, 34, 41, 46, 53, 63, 64, 76 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	7, 13, 19, 26, 33, 40, 46, 52, 59, 64, 70, 77 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	40959, 20530, 21164, 40953, 40922, 37479, 24273, 40957, 40900, 22517, 40930,
	35821 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 9, 11 };

