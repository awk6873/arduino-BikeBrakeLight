# License
The use of source code and its binary are governed by terms of use which can be found at https://neuton.ai/license. Any use in violation thereof is strictly prohibited.

# How to integrate Neuton into your firmware project 

## Include header file

Copy all files from this archive to your project and include header file:
``` C
#include "neuton.h"
```

The library contains functions to get model information such as:
* task type (regression, classification, etc.);
* neurons and weights count;
* window buffer size;
* input and output features count;
* model size and RAM usage;
* float support flag;
* quantization level.

Main functions are:
* `neuton_model_set_inputs` - to set input values;
* `neuton_model_run_inference` - to make predictions.


## Set input values

Make an array with model inputs. Inputs count and order should be the same as in the training dataset.

``` C
input_t inputs[] = {
    feature_0,
    feature_1,
    // ...
    feature_N
};
```

Pass this array to the `neuton_model_set_inputs` function. 

If the digital signal processing option was selected on the platform, you should call `neuton_model_set_inputs` multiple times for each sample to fill internal window buffer. Function will return `0` when buffer is full, this indicates that the model is ready for prediction.


##	Make prediction

When buffer is ready, you should call `neuton_model_run_inference` with two arguments:
* pointer to `index` of predicted class;
* pointer to neural net `outputs` (dimension of array can be read using the `neuton_model_outputs_count` function).

For regression task output value will be stored at `outputs[0]`.
For classification task `index` will contain class index with maximal probability, `outputs` will contain probabilities of each class. Thus, you can get predicted class probability at `outputs[index]`.

Function will return `0` on successful prediction.
``` C
if (neuton_model_set_inputs(inputs) == 0)
{
    uint16_t index;
    float* outputs;
    
    if (neuton_model_run_inference(&index, &outputs) == 0)
    {
        // code for handling prediction result
    }
}
```

## Map predicted results on the required values (for Classification task type)

Inference results are encoded (0…n). For mapping on your classes, use dictionaries `binary_target_dict_csv.csv / multi_target_dict_csv.csv`.

##	Integration with Tensorflow, ONNX

Neuton also offers additional options of integration and interaction with your model.
This archive provides you with Tensorflow and ONNX formats of the model.
You can find them in the `converted_models` folder.

##	Audio preprocessing and prediction

Since Neuton makes predictions by vector of data, sound flow must be preprocessed first. Preprocessing is made by the KWS (keyword spotting) pipeline. Here is a code example of preprocessing initialization:

``` C

#include "neuton/neuton.h"
#include "pipeline/kws/kws.h"
...

#if (MEL_SPECTROGRAM_NORMALIZATION == 1)
	float* normalizedSpectrum = neuton_model_get_inputs_ptr();	// ptr from neuton input buffer for normalized spectrum
#else
	float* normalizedSpectrum = NULL;  // no normalization
#endif	
	
	/*
	 * Create keyword spotting instance
	 */
	keyword_spotting_instance* kws = malloc(sizeof(keyword_spotting_instance));

	KeywordSpottingInstanceInit(
		kws, windowSize, windowHop, sampleRate, mfeTimeBands, mfeFreqBands, mfeShift,
		normalizedSpectrum, OnSpectrumReady, NULL);

```
Where variables `windowSize`, `windowHop`, `sampleRate`, `mfeTimeBands`, `mfeFreqBands` are set from generated header kws_config.h (neuton/model/kws_config.h) as follows:

``` C

sampleRate = kwsSamplingRate;
windowSize = kwsWindowLen;
windowHop = kwsWindowHop;
mfeFreqBands = kwsMelFreqBands;
mfeTimeBands = kwsMelTimeBands;

```

Variable `mfeShift` is 0 by default.

`OnSpectrumReady` is a call-back function, that handles a vector with spectrum data, ready for inference (description below).

Subsequent code shows reading of a test wav-file with a KWS processing. MCU firmware developers should prepare data using their own approach to sound flow.

``` C

uint32_t minDataForInference = 1000;

/*
 * Calculate minimum data length for one inference
 */
minDataForInference = kws->ts.windowSize + (kws->ts.windowHop) * (kws->mfe.melSpectrumTimeBands - 1);

/*
 * Allocate input buffer
 */
audio_sample_t* input = malloc(sizeof(audio_sample_t) * inputBufferSamplesCapacity);

/*
 * Initialize audio flow emulation
 */
if (OpenAudioFlowFromFile(filename))
	return 1;

/*
 * Process audio data
 */
for (;;)
{
	/*
	 * Read audio data block
	 */
	uint32_t samplesRead = GetNextAudioBlock(input, inputBufferSamplesCapacity);
	if (samplesRead <= 0)
		break;

	/*
	 * Process audio data block
	 */
	KeywordSpottingProcessAudio(kws, input, samplesRead);
}

```
When the KWS pipeline has data ready for inference, the `OnSpectrumReady` function is called. Here is an example of inference code:

``` C

void OnSpectrumReady(void *ctx, float* spectrum)
{
	keyword_spotting_instance* instance = (keyword_spotting_instance*)ctx;

	/*
	 * Make prediction
	 */
	uint16_t index = 0;
	float* probabilities;

#if (MEL_SPECTROGRAM_NORMALIZATION == 0)
	neuton_model_set_inputs(spectrum); // normalization is not set, setting spectrum data to neuton input buffer
#endif
	
	neuton_model_set_ready_flag();
	
	if (0 == neuton_model_run_inference(&index, &probabilities))
	{
		if (probabilities[1] >= 0.5)
			index = 1;
		else
			index = 0;
		
		fprintf(stdout, "class %u, probability %.2f\n", index, probabilities[index]);		
		fflush(stdout);
	}
}

```