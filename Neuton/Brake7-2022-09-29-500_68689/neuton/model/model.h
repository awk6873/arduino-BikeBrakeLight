/// Solution ID: 68689_Brake7-2022-09-29-500 | 2022-10-03T07:54:36Z ///

#ifndef NEUTON_MODEL_MODEL_H
#define NEUTON_MODEL_MODEL_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Model info */
#define NEUTON_MODEL_HEADER_VERSION 3
#define NEUTON_MODEL_QLEVEL 32
#define NEUTON_MODEL_FLOAT_SUPPORT 1
#define NEUTON_MODEL_TASK_TYPE 1  // binary classification
#define NEUTON_MODEL_NEURONS_COUNT 8
#define NEUTON_MODEL_WEIGHTS_COUNT 47
#define NEUTON_MODEL_INPUTS_COUNT 378
#define NEUTON_MODEL_INPUTS_COUNT_ORIGINAL 6
#define NEUTON_MODEL_INPUT_LIMITS_COUNT 378
#define NEUTON_MODEL_OUTPUTS_COUNT 2
#define NEUTON_MODEL_LOG_SCALE_OUTPUTS 0
#define NEUTON_MODEL_HAS_CLASSES_RATIO 0
#define NEUTON_MODEL_HAS_NEGPOS_RATIO 0

/* Preprocessing */
#define NEUTON_PREPROCESSING_ENABLED 1
#define NEUTON_MODEL_WINDOW_SIZE 50
#define NEUTON_DROP_ORIGINAL_FEATURES 0
#define NEUTON_BITMASK_ENABLED 1
#define NEUTON_INPUTS_IS_INTEGER 0
#define NEUTON_MODEL_SA_PRECISION 24

/* Types */
typedef float input_t;
typedef float extracted_feature_t;
typedef float coeff_t;
typedef float weight_t;
typedef float acc_signed_t;
typedef float acc_unsigned_t;
typedef uint16_t sources_size_t;
typedef uint8_t weights_size_t;
typedef uint8_t neurons_size_t;

/* Scaling */
static const input_t modelInputScaleMin[] = {
	-32868, -11610, -17840, -2195, -2288, -4943 };
static const input_t modelInputScaleMax[] = {
	32667, 16578, 32467, 1904, 4133, 3752 };

static const extracted_feature_t extractedFeaturesScaleMin[] = {
	62.25, 3.375, -3372.48, -2720, -32868, 1.002041, 686.48969, 26.200949,
	-4.5908403, -1.3060684, 1, 0, 1, 66.25, 3.375, -3396.1599, -2626, -11610,
	1.002041, 699.67383, 26.451349, -3.8903027, -1.473509, 1, 0, 0, 96.25,
	5.25, 6865.9199, 7896, -17840, 1.0060995, 1418.8549, 37.66769, -3.417237,
	-1.2231461, 3, 1, 2, 2.46875, 0.15625, -949.47998, -644, -2195, 1.002041,
	1.3044003, 1.1421034, -2.4231508, -1.8235698, 1, 0, 0, 2.15625, 0.125,
	-583.08002, -441, -2288, 1.002041, 1.2000002, 1.0954452, -3.0114758, -1.6950868,
	1, 0, 0, 2.09375, 0.125, -2279.3, -1516, -4943, 1.002041, 0.86440033, 0.92973131,
	-2.2354085, -1.819387, 1, 0, 0 };
static const extracted_feature_t extractedFeaturesScaleMax[] = {
	58463.25, 2892.8438, 1990.54, 32667, 1140, 1.0763903, 1.0215491e+08, 10107.171,
	2.4833031, 26.04744, 40, 20, 20, 14581.375, 2258.625, 2389.2, 16578, 1714,
	1.0727904, 16712806, 4088.1299, 5.6691594, 34.382149, 38, 19, 19, 43129.375,
	2561.5, 11233.36, 32467, 7980, 1.0655304, 90720648, 9524.7383, 5.7287312,
	35.436211, 34, 17, 17, 2809.125, 1022, 1038.84, 1904, 565, 1.0600307, 938592.5,
	968.80981, 2.2725422, 5.9988527, 31, 15, 16, 3816.75, 1016.5, 911.84003,
	4133, 556, 1.0581868, 2156698, 1468.5701, 3.2142777, 13.614614, 30, 15,
	15, 2844.5, 2914.5, 2566.3999, 3752, 1644, 1.0600307, 6116079.5, 2473.0708,
	4.8010187, 26.375446, 31, 16, 15 };

/* Limits */
static const uint8_t modelUsedInputsMask[] = {
	0x01, 0x00, 0x00, 0x11, 0x01, 0x08, 0x00, 0x50, 0x00, 0x00, 0x00, 0x22,
	0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x28, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x40, 0x08, 0x18, 0x00, 0x15, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00 };

/* Structure */
static const weight_t modelWeights[] = {
	0.12421595, 0.17749743, -0.07771194, -0.99900252, 0.54064733, 0.80219626,
	0.11058044, 0.41648722, -0.2964367, 0.82685524, -0.55813819, -0.92729765,
	0.4152894, -0.51777202, -0.39101231, 0.99999934, -0.29115725, 0.84219116,
	-0.45745265, 0.17193604, 0.4473815, 0.85927278, -0.1092788, -0.77020448,
	-1, 0.47689247, 0.59375, 0.55124664, -0.09375, -0.19760439, -0.48500374,
	1, -0.68401998, -0.99826181, 0.95415241, 0.17193103, -0.81556469, -0.99999988,
	0.69702506, 0.62757277, 0.24085878, 0.086563408, -0.4316121, 0.31784475,
	-0.9684459, -0.99999845, 0.099315859 };

static const sources_size_t modelLinks[] = {
	60, 89, 148, 302, 315, 316, 378, 0, 24, 195, 307, 330, 378, 28, 32, 185,
	302, 332, 367, 378, 0, 2, 378, 0, 0, 62, 93, 315, 378, 0, 1, 2, 4, 43,
	99, 378, 197, 226, 315, 328, 363, 378, 1, 4, 5, 6, 378 };

static const weights_size_t modelIntLinksBoundaries[] = {
	0, 8, 13, 22, 24, 33, 36, 46 };
static const weights_size_t modelExtLinksBoundaries[] = {
	7, 13, 20, 23, 29, 36, 42, 47 };

static const coeff_t modelFuncCoeffs[] = {
	21.093529, 14.645135, 39.999958, 19.904285, 40, 39.999985, 40, 21.296886 };
static const uint8_t modelFuncTypes[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

static const neurons_size_t modelOutputNeurons[] = { 3, 7 };

#ifdef __cplusplus
}
#endif

#endif // NEUTON_MODEL_MODEL_H

