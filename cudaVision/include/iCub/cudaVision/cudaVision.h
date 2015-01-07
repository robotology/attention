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


#ifndef __FILTERS_H__
#define __FILTERS_H__

#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>


#define MAX_KERNEL_NUM          16
#define MAX_KERNEL_LENGTH       33
#define MAX_KERNEL_RADIUS       16


#define IMUL(a, b)              __mul24(a, b)
#define IDIVUP(a, b)            ((a % b != 0) ? (a / b + 1) : (a / b))
#define IDIVDOWN(a, b)          (a / b)
#define IALIGNUP(a, b)          ((a % b != 0) ?  (a - a % b + b) : a)
#define IALIGNDOWN(a, b)        (a - (a % b))


////////////////////////////////////////////////////////////////////////////////
// GPU convolution generic functions
////////////////////////////////////////////////////////////////////////////////
extern "C" void setKernelF32Sep(float *h_Kernel, int k_Index, size_t k_Size);

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
    int k_IndexRow,
    int k_IndexCol,
    int k_Size,
    float* d_Buffer
);


extern "C" void convF32SepAdd(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_IndexRow,
    int k_IndexCol,
    int k_Size,
    float* d_Buffer
);


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
#endif  // __FILTERS_H__
