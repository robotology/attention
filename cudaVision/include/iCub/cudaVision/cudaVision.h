/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * NVIDIA Corporation and its licensors retain all intellectual property and
 * proprietary rights in and to this software and related documentation.
 * Any use, reproduction, disclosure, or distribution of this software
 * and related documentation without an express license agreement from
 * NVIDIA Corporation is strictly prohibited.
 *
 * Please refer to the applicable NVIDIA end user license agreement (EULA)
 * associated with this source code for terms and conditions that govern
 * your use of this NVIDIA software.
 *
 */


#ifndef __CUDAVISION_H__
#define __CUDAVISION_H__

#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>


#define MAX_KERNEL_NUM          16
#define MAX_KERNEL_LENGTH       64


////////////////////////////////////////////////////////////////////////////////
// GPU convolution generic functions
////////////////////////////////////////////////////////////////////////////////
extern "C" void setConvolutionKernel(float *h_Kernel, int k_Index, size_t k_Size);

////////////////////////////////////////////////////////////////////////////////
// GPU simple arethmatic functions
////////////////////////////////////////////////////////////////////////////////
extern "C" void addF32(
    float *d_Dst,
    float *d_Src1,
    float *d_Src2,
    int imageW,
    int imageH
);

////////////////////////////////////////////////////////////////////////////////
// GPU convolution Seperable
////////////////////////////////////////////////////////////////////////////////
extern "C" void convRowsF32Sep(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_Index,
    int k_Size
);

extern "C" void convColsF32Sep(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_Index,
    int k_Size
);


extern "C" void convF32Sep(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_Index,
    int k_Size,
    float* d_Buffer
);


extern "C" void convF32SepAdd(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_Index,
    int k_Size,
    float* d_Buffer
);

/*
////////////////////////////////////////////////////////////////////////////////
// GPU convolution Texture
////////////////////////////////////////////////////////////////////////////////
extern "C" void convRowsF32Tex(
    float *d_Dst,
    cudaArray *a_Src,
    int imageW,
    int imageH,
    int k_radius
);


extern "C" void convColsF32Tex(
    float *d_Dst,
    cudaArray *a_Src,
    int imageW,
    int imageH,
    int k_radius
);
*/

////////////////////////////////////////////////////////////////////////////////
// Error Handling
////////////////////////////////////////////////////////////////////////////////
#ifndef HANDLE_ERROR
static void HandleError( cudaError_t err,
                         const char *file,
                         int line ) {
    if (err != cudaSuccess) {
        printf( "%s in %s at line %d\n", cudaGetErrorString( err ),
                file, line );
        exit( EXIT_FAILURE );
    }
}
#define HANDLE_ERROR( err ) (HandleError( err, __FILE__, __LINE__ ))


#define HANDLE_NULL( a ) {if (a == NULL) { \
                            printf( "Host memory failed in %s at line %d\n", \
                                    __FILE__, __LINE__ ); \
                            exit( EXIT_FAILURE );}}
#endif
#endif  // __CUDAVISION_H__
