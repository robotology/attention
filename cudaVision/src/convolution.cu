/*
 * Copyright 1993-2010 NVIDIA Corporation.  All rights reserved.
 *
 * Please refer to the NVIDIA end user license agreement (EULA) associated
 * with this source code for terms and conditions that govern your use of
 * this software. Any use, reproduction, disclosure, or distribution of
 * this software and related documentation outside the terms of the EULA
 * is strictly prohibited.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <iCub/cudaVision/cudaVision.h>


// Assuming ROW_TILE_W, KERNEL_RADIUS_ALIGNED and dataW 
// are multiples of coalescing granularity size,
// all global memory operations are coalesced in convolutionRowGPU()
#define ROW_TILE_W              128
#define KERNEL_RADIUS_ALIGNED   16

// Assuming COLUMN_TILE_W and dataW are multiples
// of coalescing granularity size, all global memory operations 
// are coalesced in convolutionColumnGPU()
#define COLUMN_TILE_W           16
#define COLUMN_TILE_H           48


////////////////////////////////////////////////////////////////////////////////
// Convolution kernel storage
////////////////////////////////////////////////////////////////////////////////
__constant__ float c_Kernel[MAX_KERNEL_NUM][MAX_KERNEL_LENGTH];


extern "C" void setKernelF32Sep(float *h_Kernel, int k_Index, size_t k_Size)
{
    assert( k_Size <= MAX_KERNEL_LENGTH );
    assert( k_Index < MAX_KERNEL_NUM );
    cudaMemcpyToSymbol(c_Kernel[k_Index], h_Kernel, k_Size*sizeof(float));
}


#define   ADD_BLOCKDIM_X       16
#define   ADD_BLOCKDIM_Y       16
__global__ void addF32Kernel(float *C, float *A, float *B, int N)
{
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int offset = x + y * blockDim.x * gridDim.x;

    if(offset<N) 
    {
        float a = A[offset];
        float b = B[offset];
        C[offset] = a + b; 
    }
}

extern "C" void addF32(
    float *d_Dst,
    float *d_Src1,
    float *d_Src2,
    int imageW,
    int imageH)
{
    assert( imageW >= ADD_BLOCKDIM_X );
    assert( imageH >= ADD_BLOCKDIM_Y );
    dim3 dimBlock( ADD_BLOCKDIM_X, ADD_BLOCKDIM_Y );
    dim3 dimGrid( ceil(float(imageW)/float(ADD_BLOCKDIM_X)), 
                  ceil(float(imageH)/float(ADD_BLOCKDIM_Y)) );                  
    addF32Kernel<<<dimGrid, dimBlock>>>(d_Dst, d_Src1, d_Src2, imageW*imageH);
}



/**
 * Seperable Convlution
 */

////////////////////////////////////////////////////////////////////////////////
// Row convolution filter
////////////////////////////////////////////////////////////////////////////////
__global__ void convRowsF32SepKernel(
    float *d_Result,
    float *d_Data,
    int dataW,
    int dataH,
    int k_Index,    
    int k_radius)
{
    //Data cache
    __shared__ float data[MAX_KERNEL_RADIUS + ROW_TILE_W + MAX_KERNEL_RADIUS];

    //Current tile and apron limits, relative to row start
    const int tileStart = IMUL(blockIdx.x, ROW_TILE_W);
    const int tileEnd = tileStart + ROW_TILE_W - 1;
    const int apronStart = tileStart - k_radius;
    const int apronEnd = tileEnd + k_radius;

    //Clamp tile and apron limits by image borders
    const int tileEndClamped = min(tileEnd, dataW - 1);
    const int apronStartClamped = max(apronStart, 0);
    const int apronEndClamped = min(apronEnd, dataW - 1);

    //Row start index in d_Data[]
    const int rowStart = IMUL(blockIdx.y, dataW);

    //Aligned apron start. Assuming dataW and ROW_TILE_W are multiples 
    //of half-warp size, rowStart + apronStartAligned is also a 
    //multiple of half-warp size, thus having proper alignment 
    //for coalesced d_Data[] read.
    const int apronStartAligned = tileStart - KERNEL_RADIUS_ALIGNED;

    const int loadPos = apronStartAligned + threadIdx.x;
    //Set the entire data cache contents
    //Load global memory values, if indices are within the image borders,
    //or initialize with zeroes otherwise
    if(loadPos >= apronStart)
    {
        const int smemPos = loadPos - apronStart;

        data[smemPos] = 
            ((loadPos >= apronStartClamped) && (loadPos <= apronEndClamped)) ?
            d_Data[rowStart + loadPos] : 0;
    }


    //Ensure the completness of the loading stage
    //because results, emitted by each thread depend on the data,
    //loaded by another threads
    __syncthreads();
    const int writePos = tileStart + threadIdx.x;
    
    //Assuming dataW and ROW_TILE_W are multiples of half-warp size,
    //rowStart + tileStart is also a multiple of half-warp size,
    //thus having proper alignment for coalesced d_Result[] write.
    if(writePos <= tileEndClamped)
    {
        const int smemPos = writePos - apronStart;
        float sum = 0;

        for(int k = -k_radius; k <= k_radius; k++)
            sum += data[smemPos + k] * c_Kernel[k_Index][k_radius - k];

        d_Result[rowStart + writePos] = sum;
    }
}


extern "C" void convRowsF32Sep(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_Index,
    int k_Size)
{
    int k_radius = ((k_Size % 2) == 0) ? k_Size/2 : (k_Size-1)/2;
    dim3 blocks(IDIVUP(imageW, ROW_TILE_W), imageH);
    dim3 threads(KERNEL_RADIUS_ALIGNED + ROW_TILE_W + k_radius);	// 16 128 8

    convRowsF32SepKernel<<<blocks, threads>>>(
        d_Dst,
        d_Src,
        imageW,
        imageH,
        k_Index,
        k_radius);
}



////////////////////////////////////////////////////////////////////////////////
// Column convolution filter
////////////////////////////////////////////////////////////////////////////////
__global__ void convColsF32SepKernel(
    float *d_Result,
    float *d_Data,
    int dataW,
    int dataH,
    int k_index,
    int k_radius,
    int smemStride,
    int gmemStride)
{
    //Data cache
    __shared__ float data[COLUMN_TILE_W * (MAX_KERNEL_RADIUS + COLUMN_TILE_H + MAX_KERNEL_RADIUS)];

    //Current tile and apron limits, in rows
    const int tileStart = IMUL(blockIdx.y, COLUMN_TILE_H);
    const int tileEnd = tileStart + COLUMN_TILE_H - 1;
    const int apronStart = tileStart - k_radius;
    const int apronEnd = tileEnd + k_radius;

    //Clamp tile and apron limits by image borders
    const int tileEndClamped = min(tileEnd, dataH - 1);
    const int apronStartClamped = max(apronStart, 0);
    const int apronEndClamped = min(apronEnd, dataH - 1);

    //Current column index
    const int columnStart = IMUL(blockIdx.x, COLUMN_TILE_W) + threadIdx.x;

    //Shared and global memory indices for current column
    int smemPos = IMUL(threadIdx.y, COLUMN_TILE_W) + threadIdx.x;
    int gmemPos = IMUL(apronStart + threadIdx.y, dataW) + columnStart;
    
    //Cycle through the entire data cache
    //Load global memory values, if indices are within the image borders,
    //or initialize with zero otherwise
    for(int y = apronStart + threadIdx.y; y <= apronEnd; y += blockDim.y)
    {
        data[smemPos] = 
        ((y >= apronStartClamped) && (y <= apronEndClamped)) ? 
        d_Data[gmemPos] : 0;
        smemPos += smemStride;
        gmemPos += gmemStride;
    }

    //Ensure the completness of the loading stage
    //because results, emitted by each thread depend on the data, 
    //loaded by another threads
    __syncthreads();
    
    //Shared and global memory indices for current column
    smemPos = IMUL(threadIdx.y + k_radius, COLUMN_TILE_W) + threadIdx.x;
    gmemPos = IMUL(tileStart + threadIdx.y , dataW) + columnStart;
    
    //Cycle through the tile body, clamped by image borders
    //Calculate and output the results
    for(int y = tileStart + threadIdx.y; y <= tileEndClamped; y += blockDim.y)
    {
        float sum = 0;
        for(int k = -k_radius; k <= k_radius; k++)
            sum += 
                data[smemPos + IMUL(k, COLUMN_TILE_W)] *
                c_Kernel[k_index][k_radius - k];

        d_Result[gmemPos] = sum;
        smemPos += smemStride;
        gmemPos += gmemStride;
    }
}


extern "C" void convColsF32Sep(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_Index,
    int k_Size)
{
    int k_radius = ((k_Size % 2) == 0) ? k_Size/2 : (k_Size-1)/2;
    dim3 blocks(IDIVUP(imageW, COLUMN_TILE_W), IDIVUP(imageH, COLUMN_TILE_H));
    dim3 threads(COLUMN_TILE_W, 8);

    convColsF32SepKernel<<<blocks, threads>>>(
        d_Dst,
        d_Src,
        imageW,
        imageH,        
        k_Index,
        k_radius,
        COLUMN_TILE_W*threads.y,
        imageW*threads.y);
}


extern "C" void convF32Sep(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_IndexRow,
    int k_IndexCol,
    int k_Size,
    float* d_Buffer)
{

    if(d_Buffer == NULL)
    {
        HANDLE_ERROR( cudaMalloc((void **)&d_Buffer, imageW*imageH*sizeof(float)) );   
        convRowsF32Sep(d_Buffer, d_Src, imageW, imageH, k_IndexRow, k_Size);
        convColsF32Sep(d_Dst, d_Buffer, imageW, imageH, k_IndexCol, k_Size);
        HANDLE_ERROR( cudaFree(d_Buffer ) );
    }
    else
    {
        convRowsF32Sep(d_Buffer, d_Src, imageW, imageH, k_IndexRow, k_Size);
        convColsF32Sep(d_Dst, d_Buffer, imageW, imageH, k_IndexCol, k_Size);
    }
}


extern "C" void convF32SepAdd(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_IndexRow,
    int k_IndexCol,
    int k_Size,
    float* d_Buffer)
{

    if(d_Buffer == NULL)
    {
        HANDLE_ERROR( cudaMalloc((void **)&d_Buffer, imageW*imageH*sizeof(float)) );   
        convRowsF32Sep(d_Buffer, d_Src, imageW, imageH, k_IndexRow, k_Size);
        HANDLE_ERROR( cudaDeviceSynchronize() );
        convColsF32Sep(d_Dst, d_Src, imageW, imageH, k_IndexCol, k_Size);
        addF32(d_Dst, d_Dst, d_Buffer, imageW, imageH); 
        HANDLE_ERROR( cudaDeviceSynchronize() );
        HANDLE_ERROR( cudaFree(d_Buffer ) );
    }
    else
    {
        convRowsF32Sep(d_Buffer, d_Src, imageW, imageH, k_IndexRow, k_Size);
        HANDLE_ERROR( cudaDeviceSynchronize() );
        convColsF32Sep(d_Dst, d_Src, imageW, imageH, k_IndexCol, k_Size);
        HANDLE_ERROR( cudaDeviceSynchronize() );
        addF32(d_Dst, d_Dst, d_Buffer, imageW, imageH);        
        HANDLE_ERROR( cudaDeviceSynchronize() );
    }
}



