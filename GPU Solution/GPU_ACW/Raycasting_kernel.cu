/*
*   Built upon bicubicTexture sample and RT Lab
*/

    #pragma region includes
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <helper_math.h>
#include <helper_cuda.h>
#include <device_launch_parameters.h>
#include <math.h>
#include <curand.h>
#include <curand_kernel.h>
#include <iostream>
#include <sphere.h>
#include <hitable.h>
#include <hitable_list.h>
#include <vec3.h>
#include <Ray.h>
    #pragma endregion

typedef unsigned int uint;
typedef unsigned char uchar;


    #pragma region OpenGL

cudaArray* d_imageArray = 0;

extern "C"
void initTexture(int imageWidth, int imageHeight, uchar * h_data)
{
    cudaResourceDesc texRes;
    memset(&texRes, 0, sizeof(cudaResourceDesc));

    texRes.resType = cudaResourceTypeArray;
    texRes.res.array.array = d_imageArray;
}

extern "C"
void freeTexture()
{
    checkCudaErrors(cudaFreeArray(d_imageArray));
}

#define checkCudaErrors(val) check_cuda((val), #val, __FILE__, __LINE__)

void check_cuda(cudaError_t result, char const* const func, const char* const file, int const line) {
    if (result) {
        std::cerr << "CUDA error = " << static_cast<unsigned int>(result) << " at " << file << ":" << line << " '" << func << "' \n";
        // Make sure we call CUDA Device Reset before exiting
        cudaDeviceReset();
        exit(99);
    }
}

    #pragma endregion

    #pragma region Random Number Generator
// Random Number generation
// http://ianfinlayson.net/class/cpsc425/notes/cuda-random
# define MAX_RANDOM 99
__device__ void random(int* result, int seed) {
    __shared__ curandState_t state;
    curand_init(seed, 0, 0, &state);
    *result = curand(&state) % MAX_RANDOM;
}
    #pragma endregion

# define PARTICLE_COUNT 20

static hitable** d_list;
static hitable** d_world;

__device__ vec3 castRay(const ray& r, hitable** world) {
    hit_record rec;
    if ((*world)->hit(r, 0.0, FLT_MAX, rec))
    {
        return 0.5f * vec3(rec.normal.x() + 1.0f, rec.normal.y() + 1.0f, rec.normal.z() + 1.0f);
    }
    else
    {
        return vec3(0, 0, 0);
    }
}

__global__ void free_world(hitable** d_list, hitable** d_world)
{
    delete* (d_list);
    delete* (d_list + 1);
    delete* d_world;
}

__global__ void create_world(hitable** d_list, hitable** d_world)
{
    if (threadIdx.x == 0 && blockIdx.x == 0) {

        int randomNumbers[PARTICLE_COUNT * 6];

        for (int i = 0; i < PARTICLE_COUNT * 6; i++) {
            int* num = &randomNumbers[i];
            random(num, i+1);
            //printf("%d\n", randomNumbers[i]);
        }

        for (int i = 0; i < PARTICLE_COUNT; i++) {
            int randomIndex = i * 6;
            vec3 loc = vec3(randomNumbers[randomIndex] - 50, randomNumbers[randomIndex + 1]- 50, randomNumbers[randomIndex + 2] * -1);
            vec3 vel = vec3(randomNumbers[randomIndex + 3] - 50, randomNumbers[randomIndex + 4] - 50, randomNumbers[randomIndex + 5] -50)/10;
            //printf("%d", randomNumbers[i]);
            *(d_list + i) = new sphere(loc, vel);
        }

        *d_world = new hitable_list(d_list, PARTICLE_COUNT);
    }
}

//
//__device__ vec3 collisionReVel(sphere particle) {
//    vec3 a = particle.vel - vel;
//    vec3 iniMom = particle.vel + vel;
//    vec3 v2After = (iniMom - a) / 2;
//    return v2After + a;
//}

__global__ void d_gravity(hitable** world) {
    (*world)->applyGravity();
}

__global__ void d_letChaos(hitable** world) {
    __shared__ int randomNumbers[PARTICLE_COUNT * 3];

    if (threadIdx.x == 0 && blockIdx.x == 0) {
        for (int i = 0; i < PARTICLE_COUNT * 3; i++) {
            int* num = &randomNumbers[i];
            random(num, i + 1);
        }
    }
    __syncthreads();

    int randomIndex = threadIdx.x * 3;
    vec3 newVel = vec3(randomNumbers[randomIndex] - 50, randomNumbers[randomIndex + 1] - 50, randomNumbers[randomIndex + 2] - 50) / 10;
    (*world)->letChaos(threadIdx.x, newVel);
}

__global__ void d_moveParticles(hitable** world) {
    (*world)->reVel(threadIdx.x);
    (*world)->move(threadIdx.x);
}

__global__ void d_raycast(uchar4* d_output, uint width, uint height, hitable** d_world)
{
    uint x = blockIdx.x * blockDim.x + threadIdx.x;
    uint y = blockIdx.y * blockDim.y + threadIdx.y;
    uint i = y * width + x;

    float u = float(x) / float(width);
    float v = float(y) / float(height);

    u = 2.0 * u - 1;
    v = -(2.0 * v - 1);

    u *= float(width) / float(height);

    u *= 2.0;
    v *= 2.0;

    vec3 eye = vec3(0, 0, 1.5);
    float distFrEye2Img = 1.0;

    if ((x < width) && (y < height))
    {
        //for each pixel
        vec3 pixelPos = vec3(u, v, eye.z() - distFrEye2Img);
        //fire a ray:
        ray r;
        r.O = eye;

        r.Dir = pixelPos - eye;    //view direction along negtive z-axis
        vec3 col = castRay(r, d_world);
        float red = col.x();
        float green = col.y();
        float blue = col.z();
        d_output[i] = make_uchar4(red * 255, green * 255, blue * 255, 0);
    }
}


// render image using CUDA
extern "C"
void render(int width, int height, dim3 blockSize, dim3 gridSize, uchar4 * output)
{
    _sleep(10);
    d_moveParticles << <1, PARTICLE_COUNT >> > (d_world);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());

    d_raycast << <gridSize, blockSize >> > (output, width, height, d_world);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
}

extern "C"
void applyGravity() {
    checkCudaErrors(cudaDeviceSynchronize());
    d_gravity << <1, PARTICLE_COUNT >> > (d_world);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
}

extern "C"
void letChaos() {
    checkCudaErrors(cudaDeviceSynchronize());
    d_letChaos << <1, PARTICLE_COUNT >> > (d_world);
    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
}

extern "C"
void particlesInitialise(int width, int height, dim3 blockSize, dim3 gridSize)
{
    checkCudaErrors(cudaMalloc((void**)&d_list, 2 * sizeof(hitable*)));
    checkCudaErrors(cudaMalloc((void**)&d_world, sizeof(hitable*)));

    
    create_world << <1, 1>> > (d_list, d_world);

    checkCudaErrors(cudaGetLastError());
    checkCudaErrors(cudaDeviceSynchronize());
}

