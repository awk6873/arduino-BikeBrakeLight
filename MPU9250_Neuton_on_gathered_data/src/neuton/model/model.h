/// Solution ID: 68695_Brake9-2022-09-29-I32-750 | 2022-10-03T10:35:01Z ///

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
#define NEUTON_MODEL_NEURONS_COUNT 6
#define NEUTON_MODEL_WEIGHTS_COUNT 31
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
#define NEUTON_INPUTS_IS_INTEGER 1
#define NEUTON_MODEL_SA_PRECISION 15

/* Types */
typedef int16_t input_t;
typedef int32_t extracted_feature_t;
typedef float coeff_t;
typedef float weight_t;
typedef float acc_signed_t;
typedef float acc_unsigned_t;
typedef uint16_t sources_size_t;
typedef uint8_t weights_size_t;
typedef uint8_t neurons_size_t;

/* Scaling */
static const input_t modelInputScaleMin[] = {
	-29480, -11610, -17840, -2195, -2288, -4943 };
static const input_t modelInputScaleMax[] = {
	32668, 16578, 32467, 1904, 4133, 3752 };

static const extracted_feature_t extractedFeaturesScaleMin[] = {
	3436, 72, -103055360, -2548, -29480, 65618, 914, 990848, 15126, 6726, 1,
	1, 0, 3452, 184, -106975232, -2374, -11610, 65618, 909, 987936, 16352,
	6326, 1, 0, 0, 5488, 280, 223240192, 7988, -17840, 66418, 1844, 1407392,
	15041, 7586, 11, 5, 6, 123, 12, -25821696, -636, -2195, 65618, 1, 42472,
	24785, 4739, 1, 0, 0, 126, 9, -15460352, -269, -2288, 65618, 1, 38956,
	22616, 5433, 1, 0, 0, 106, 9, -64182272, -1275, -4943, 65618, 1, 37264,
	24244, 4984, 1, 0, 0 };
static const extracted_feature_t extractedFeaturesScaleMax[] = {
	3524784, 84728, 38950912, 32668, 1116, 69578, 100820992, 329023488, 44231,
	125208, 53, 26, 27, 767064, 168256, 60979200, 16578, 1706, 69506, 13795328,
	121706496, 57797, 195208, 52, 26, 26, 1440140, 168444, 304865280, 32467,
	7968, 69216, 83154944, 298811392, 60072, 219816, 48, 24, 24, 48510, 54040,
	28332032, 1904, 468, 68632, 1197504, 35858432, 43362, 41634, 40, 21, 21,
	187176, 38428, 23297536, 4133, 431, 68410, 1522400, 40430592, 46524, 56512,
	37, 18, 21, 110331, 178540, 71448576, 3752, 1592, 68334, 7357440, 88881152,
	41426, 43964, 36, 20, 19 };

/* Limits */
static const uint8_t modelUsedInputsMask[] = {
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x20, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00,
	0x00, 0x00, 0x08, 0x00, 0x00, 0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
	0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x22, 0x00,
	0x00, 0x10, 0x00, 0x10, 0x00, 0x00 };

/* Structure */
static const weight_t modelWeights[] = {
	0.11983839, 0.43215555, -1, 0.77843225, 0.57709289, -0.14084244, 1, 0.8250702,
	-0.79002458, 0.99945933, 1, -0.54000592, -0.77097797, -1, -0.99999976,
	0.79150808, -0.34871382, 0.092423916, -1, 0.73199332, 0, 0.87958312, -0.66041613,
	1, -0.99980414, -0.85461456, 0.40946022, 0.35313129, 0.9375, 0.93421215,
	-0.057842381 };

static const sources_size_t modelLinks[] = {
	138, 328, 452, 465, 492, 528, 0, 274, 278, 359, 508, 528, 0, 85, 307, 452,
	469, 528, 1, 2, 528, 0, 48, 154, 388, 452, 457, 528, 0, 4, 528 };

static const weights_size_t modelIntLinksBoundaries[] = { 0, 7, 13, 20, 21, 30 };
static const weights_size_t modelExtLinksBoundaries[] = {
	6, 12, 18, 21, 28, 31 };

static const coeff_t modelFuncCoeffs[] = {
	40, 40, 40, 40, 39.999985, 39.998047 };
static const uint8_t modelFuncTypes[] = { 0, 0, 0, 0, 0, 0 };

static const neurons_size_t modelOutputNeurons[] = { 5, 3 };

#ifdef __cplusplus
}
#endif

#endif // NEUTON_MODEL_MODEL_H

