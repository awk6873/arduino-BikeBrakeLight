/// Solution ID: 68659_Brake-2022-09-29 | 2022-10-01T06:27:56Z ///

#ifndef NEUTON_FFT_CONFIG_H
#define NEUTON_FFT_CONFIG_H

#include "model.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __GNUC__
#define SUPRESS_WARNING_VARIABLE_IS_NOT_USED __attribute__ ((unused))
#else
#define SUPRESS_WARNING_VARIABLE_IS_NOT_USED
#endif

static const uint16_t fftWindowSize = 64;
static const uint16_t fftLogN = 6;

static input_t SUPRESS_WARNING_VARIABLE_IS_NOT_USED fftWindow[64];

static const uint16_t fftReversedBitIndexArray[] = {
	0, 32, 16, 48, 8, 40, 24, 56, 4, 36, 20, 52, 12, 44, 28, 60, 2, 34, 18,
	50, 10, 42, 26, 58, 6, 38, 22, 54, 14, 46, 30, 62, 1, 33, 17, 49, 9, 41,
	25, 57, 5, 37, 21, 53, 13, 45, 29, 61, 3, 35, 19, 51, 11, 43, 27, 59, 7,
	39, 23, 55, 15, 47, 31, 63 };

static const input_t fftSinTable[] = {
	0, 0.098017141, 0.19509032, 0.29028466, 0.38268346, 0.47139674, 0.55557024,
	0.63439333, 0.70710677, 0.77301043, 0.83146966, 0.88192129, 0.9238795,
	0.95694035, 0.98078531, 0.99518472 };

static const input_t fftTgTable[] = {
	0, 0.049126852, 0.098491408, 0.14833599, 0.19891237, 0.25048697, 0.30334669,
	0.35780576, 0.41421357, 0.47296476, 0.53451115, 0.59937698, 0.66817868,
	0.74165058, 0.82067883, 0.90634716 };

#ifdef __cplusplus
}
#endif

#endif // NEUTON_FFT_CONFIG_H

