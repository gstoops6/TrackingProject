#include "ParticleFilter.h"
#include <curand_kernel.h>

__global__ void setup_kernel(curandState *state, unsigned long seed) {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    // Each thread gets the same seed, a different sequence number, and 0 offset
    curand_init(seed, id, 0, &state[id]);
}

__global__ void generate_kernel(curandState *state, float *results) {
    int id = threadIdx.x + blockIdx.x * blockDim.x;

    // Copy state to local register memory for faster access
    curandState localState = state[id];

    // Generate a uniform float between 0.0 and 1.0
    float x = curand_uniform(&localState);

    // Save updated state back to global memory
    state[id] = localState;
    results[id] = x;
}

void get_generator(ParticleFilter *pf){

}