/// Solution ID: 68664_Brake5-2022-09-29 | 2022-10-01T11:05:35Z ///

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
#define NEUTON_MODEL_NEURONS_COUNT 5
#define NEUTON_MODEL_WEIGHTS_COUNT 24
#define NEUTON_MODEL_INPUTS_COUNT 528
#define NEUTON_MODEL_INPUTS_COUNT_ORIGINAL 6
#define NEUTON_MODEL_INPUT_LIMITS_COUNT 528
#define NEUTON_MODEL_OUTPUTS_COUNT 2
#define NEUTON_MODEL_LOG_SCALE_OUTPUTS 0
#define NEUTON_MODEL_HAS_CLASSES_RATIO 0
#define NEUTON_MODEL_HAS_NEGPOS_RATIO 0

/* Preprocessing */
#define NEUTON_PREPROCESSING_ENABLED 1
#define NEUTON_MODEL_WINDOW_SIZE 75
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
	53.6875, 1.125, -3145.0134, -2548, -32868, 1.0012335, 913.42499, 30.222921,
	-4.3058944, -1.3584734, 1, 1, 0, 107.875, 5.75, -3264.6667, -2374, -11610,
	1.0012335, 908.37335, 30.139233, -4.0042143, -1.4554456, 1, 0, 0, 171.5,
	8.75, 6812.7466, 7988, -17840, 1.0133812, 1844.6904, 42.94986, -4.3213344,
	-1.1482013, 11, 5, 6, 3.84375, 0.375, -788.02667, -636, -2195, 1.0012335,
	1.5406208, 1.2412175, -1.953558, -1.8430678, 1, 0, 0, 3.9375, 0.28125,
	-471.81332, -269, -2288, 1.0012335, 1.204623, 1.0975531, -2.4710736, -1.6736985,
	1, 0, 0, 3.3125, 0.28125, -1958.7333, -1275, -4943, 1.0012335, 1.1189332,
	1.0577964, -2.0928383, -1.7836225, 1, 0, 0 };
static const extracted_feature_t extractedFeaturesScaleMax[] = {
	42788.125, 1323.875, 1188.6934, 32667, 1116, 1.0611868, 1.0086007e+08,
	10042.911, 2.7970145, 27.555254, 53, 26, 27, 23970.75, 5258, 1860.9867,
	16578, 1706, 1.0601012, 13801225, 3715, 6.1038618, 44.644764, 52, 26, 26,
	45004.375, 5263.875, 9303.8398, 32467, 7968, 1.0546378, 83197800, 9121.2832,
	6.6609464, 50.63578, 47, 24, 23, 1515.9375, 1688.75, 864.62665, 1904, 468,
	1.0480025, 1198014.1, 1094.5383, 2.5857015, 7.3114948, 41, 20, 21, 5849.25,
	1200.875, 710.98669, 4133, 431, 1.0502241, 1522961.8, 1234.0834, 3.3523076,
	10.770352, 43, 22, 21, 3447.8438, 5579.375, 2180.48, 3752, 1592, 1.0480025,
	7361197, 2713.1526, 2.1076601, 7.7660828, 41, 20, 21 };

/* Limits */
static const uint8_t modelUsedInputsMask[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x01, 0x04, 0x00, 0x00, 0x20, 0x00,
	0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x42, 0x00,
	0x00, 0x10, 0x20, 0x00, 0x00, 0x00 };

/* Structure */
static const weight_t modelWeights[] = {
	0.22581907, 0.39170277, -1, 0.76155061, 0.56408226, -0.14937592, 0.58553451,
	0, 1, -0.74248159, 0.68497086, 0.53865725, -0.28339294, 0.2552681, -0.32518816,
	-0.74957597, -0.82606918, -0.89389431, 0.99999988, -0.21262754, 0.11524487,
	-1, 0.50466233, 0 };

static const sources_size_t modelLinks[] = {
	125, 328, 452, 465, 492, 528, 0, 528, 0, 144, 195, 308, 407, 470, 528,
	0, 154, 181, 452, 501, 528, 2, 3, 528 };

static const weights_size_t modelIntLinksBoundaries[] = { 0, 7, 9, 16, 23 };
static const weights_size_t modelExtLinksBoundaries[] = { 6, 8, 15, 21, 24 };

static const coeff_t modelFuncCoeffs[] = { 40, 40, 40, 40, 40 };
static const uint8_t modelFuncTypes[] = { 0, 0, 0, 0, 0 };

static const neurons_size_t modelOutputNeurons[] = { 1, 4 };

#ifdef __cplusplus
}
#endif

#endif // NEUTON_MODEL_MODEL_H

