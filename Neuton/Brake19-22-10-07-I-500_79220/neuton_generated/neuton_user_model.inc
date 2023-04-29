/* 2023-04-27T06:57:04Z */

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
	-32749, -6684, 10864, 24308, 11915, 3573, 19466, 5958, -20840, -25196,
	5544, 17805, 10004, 1327, -22658, -191, -32757, 26840, 1633, 16052, -32768,
	-28570, 20190, 10611, 9353, -10309, -24517, 25358, 1409, 24327, -18250,
	-25362, -15600, 25642, 4356, -12847, 21518, -17599, -13014, -23801, 24040,
	-26807, 17479, 8430, 32764, 31280, -31910, -1821 };

static const neuton_sources_num_t MODEL_LINKS[] = {
	1, 138, 374, 379, 497, 504, 0, 90, 162, 363, 476, 500, 504, 144, 314, 342,
	363, 379, 386, 504, 2, 39, 160, 314, 419, 504, 1, 3, 504, 55, 168, 262,
	386, 464, 504, 1, 3, 5, 0, 57, 140, 319, 504, 0, 2, 5, 6, 504 };

static const neuton_weights_num_t MODEL_INTERNAL_LINKS_BOUNDARIES[] = {
	0, 7, 13, 21, 28, 29, 38, 47 };

static const neuton_weights_num_t MODEL_EXTERNAL_LINKS_BOUNDARIES[] = {
	6, 13, 20, 26, 29, 35, 43, 48 };

static const neuton_coeff_t MODEL_ACTIVATION_COEFFS[] = {
	40960, 17431, 40956, 26317, 16469, 21245, 33297, 28161 };

static const neuton_neurons_num_t MODEL_OUTPUT_NEURONS[] = { 7, 4 };

