# Neuton Inference Runner Executable

Neuton Inference Runner is an executable containing a user-specific Neuton ML solution. With its help, the user can validate the results of the inference of the model even without implementation in the Edge device!

Neuton Inference Runner сompiled for the following OS:
 * `neuton_inference_runner_linux` - Linux based system, e.g. ubuntu:20.04;
 * `neuton_inference_runner_win` - 64-bit Windows based system;
 * `neuton_inference_runner_mac` - will be added soon!

# How to use

Neuton Inference Runner has user CLI interface for executing various commands.

To see `help` you can run `neuton_inference_runner` without any arguments:
``` C

.\artifacts\inference_runner>neuton_inference_runner_linux

DESCRIPTION
        Neuton Inference Runner Executable

SYNOPSIS
        neuton_inference_runner_<Os> info
        neuton_inference_runner_<Os> inference <dataset> [-s <filename>]
        neuton_inference_runner_<Os> metrics <dataset>

OPTIONS
        info                      print Neuton solution info

        inference
            <dataset>             filename of the user dataset
            -s, --save            save the inference result to the <filename>

        metrics
            <dataset>             filename of the user dataset with target

LICENSE
        BELLINTEGRATOR
```

## Get Neuton solution information

Use `info` command to see Neuton solution information that compiled in Neuton Inference Runner:

``` C

.\artifacts\inference_runner>neuton_inference_runner_linux info
      Solution name: My_Neuton_Project
    Model bit depth: 16 bits
      Float support: 1
          Task type: Binary classification
         Input type: float
Unique inputs count: 10
        Window size: 1
 Input scaling type: unified
      Outputs count: 2
      Neurons count: 11
  Model flash usage: 241 bytes

```

## Run inference

Use `inference` command to run inference on the provided CSV dataset.

Supported two kinds of datasets: `CSV` text file format and `Neuton dataset` binary file format (with corresponding filename extensions '.csv' and '.bin'). 
The `CSV` dataset should have a header and contains only number fields separated by comma.

Inference result will be printed to the screen. Additional params `-s <filename>` (or `--save <fileneme>`) stores the inference results to the CSV file.

usage examples:
```
.\artifacts\inference_runner>neuton_inference_runner_linux inference test.csv
```
```
.\artifacts\inference_runner>neuton_inference_runner_linux inference test.csv -s result.csv
```

## Get metrics

Use `metrics` command to calculate validation metrics. It is only possible if the provided dataset contains target values. Dataset shoud have a header with a target column named `target`.

usage examples:
```
.\artifacts\inference_runner>neuton_inference_runner_linux metrics test.csv
```

