#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <cublas.h>
#include <curand.h>
#include <cuda_device_runtime_api.h>
#include <driver_types.h>
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>





#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    size_t state_size;
    size_t cov_size;

    float *x_prior;
    float *cov_prior;
    float *x_posterior;
    float *cov_posterior;
}ParticleFilter;



#ifdef __cplusplus
}
#endif
#endif