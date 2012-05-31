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

////////////////////////////////////////////////////////////////////////////////
// Convolution kernel storage
////////////////////////////////////////////////////////////////////////////////
__constant__ float c_Kernel[MAX_KERNEL_NUM][MAX_KERNEL_LENGTH];


extern "C" void setConvolutionKernel(float *h_Kernel, int k_Index, size_t k_Size)
{
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


// Row convolution filter
#define   ROWS_BLOCKDIM_X       32//16
#define   ROWS_BLOCKDIM_Y       4
#define   ROWS_RESULT_STEPS     5//8
#define   ROWS_HALO_STEPS       1

__global__ void convRowsF32SepKernel(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int pitch,
    int k_index,
    int k_radius)
{
    __shared__ float s_Data[ROWS_BLOCKDIM_Y][(ROWS_RESULT_STEPS + 2 * ROWS_HALO_STEPS) * ROWS_BLOCKDIM_X];

    //Offset to the left halo edge
    const int baseX = (blockIdx.x * ROWS_RESULT_STEPS - ROWS_HALO_STEPS) * ROWS_BLOCKDIM_X + threadIdx.x;
    const int baseY = blockIdx.y * ROWS_BLOCKDIM_Y + threadIdx.y;

    d_Src += baseY * pitch + baseX;
    d_Dst += baseY * pitch + baseX;

    //Load main data
    #pragma unroll
    for(int i = ROWS_HALO_STEPS; i < ROWS_HALO_STEPS + ROWS_RESULT_STEPS; i++)
        s_Data[threadIdx.y][threadIdx.x + i * ROWS_BLOCKDIM_X] = d_Src[i * ROWS_BLOCKDIM_X];

    //Load left halo
    #pragma unroll
    for(int i = 0; i < ROWS_HALO_STEPS; i++)
        s_Data[threadIdx.y][threadIdx.x + i * ROWS_BLOCKDIM_X] = (baseX >= -i * ROWS_BLOCKDIM_X ) ? d_Src[i * ROWS_BLOCKDIM_X] : 0;

    //Load right halo
    #pragma unroll
    for(int i = ROWS_HALO_STEPS + ROWS_RESULT_STEPS; i < ROWS_HALO_STEPS + ROWS_RESULT_STEPS + ROWS_HALO_STEPS; i++)
        s_Data[threadIdx.y][threadIdx.x + i * ROWS_BLOCKDIM_X] = (imageW - baseX > i * ROWS_BLOCKDIM_X) ? d_Src[i * ROWS_BLOCKDIM_X] : 0;

    //Compute and store results
    __syncthreads();
    #pragma unroll
    for(int i = ROWS_HALO_STEPS; i < ROWS_HALO_STEPS + ROWS_RESULT_STEPS; i++){
        float sum = 0;

        #pragma unroll
        for(int j = -k_radius; j <= k_radius; j++)
            sum += c_Kernel[k_index][k_radius - j] * s_Data[threadIdx.y][threadIdx.x + i * ROWS_BLOCKDIM_X + j];

        d_Dst[i * ROWS_BLOCKDIM_X] = sum;
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
    assert( ROWS_BLOCKDIM_X * ROWS_HALO_STEPS >= k_radius );
    assert( imageW % (ROWS_RESULT_STEPS * ROWS_BLOCKDIM_X) == 0 );
    assert( imageH % ROWS_BLOCKDIM_Y == 0 );

    dim3 blocks(imageW / (ROWS_RESULT_STEPS * ROWS_BLOCKDIM_X), imageH / ROWS_BLOCKDIM_Y);
    dim3 threads(ROWS_BLOCKDIM_X, ROWS_BLOCKDIM_Y);

    convRowsF32SepKernel<<<blocks, threads>>>(
        d_Dst,
        d_Src,
        imageW,
        imageH,
        imageW,
        k_Index,
        k_radius);
    //cutilCheckMsg("convolutionRowsKernel() execution failed\n");
}




// Column convolution filter
#define   COLUMNS_BLOCKDIM_X        16
#define   COLUMNS_BLOCKDIM_Y        16 //8
#define   COLUMNS_RESULT_STEPS      5 //8
#define   COLUMNS_HALO_STEPS        1

__global__ void convColsF32SepKernel(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int pitch,
    int k_index,
    int k_radius)
{
    __shared__ float s_Data[COLUMNS_BLOCKDIM_X][(COLUMNS_RESULT_STEPS + 2 * COLUMNS_HALO_STEPS) * COLUMNS_BLOCKDIM_Y + 1];

    //Offset to the upper halo edge
    const int baseX = blockIdx.x * COLUMNS_BLOCKDIM_X + threadIdx.x;
    const int baseY = (blockIdx.y * COLUMNS_RESULT_STEPS - COLUMNS_HALO_STEPS) * COLUMNS_BLOCKDIM_Y + threadIdx.y;
    d_Src += baseY * pitch + baseX;
    d_Dst += baseY * pitch + baseX;

    //Main data
    #pragma unroll
    for(int i = COLUMNS_HALO_STEPS; i < COLUMNS_HALO_STEPS + COLUMNS_RESULT_STEPS; i++)
        s_Data[threadIdx.x][threadIdx.y + i * COLUMNS_BLOCKDIM_Y] = d_Src[i * COLUMNS_BLOCKDIM_Y * pitch];

    //Upper halo
    #pragma unroll
    for(int i = 0; i < COLUMNS_HALO_STEPS; i++)
        s_Data[threadIdx.x][threadIdx.y + i * COLUMNS_BLOCKDIM_Y] = (baseY >= -i * COLUMNS_BLOCKDIM_Y) ? d_Src[i * COLUMNS_BLOCKDIM_Y * pitch] : 0;

    //Lower halo
    #pragma unroll
    for(int i = COLUMNS_HALO_STEPS + COLUMNS_RESULT_STEPS; i < COLUMNS_HALO_STEPS + COLUMNS_RESULT_STEPS + COLUMNS_HALO_STEPS; i++)
        s_Data[threadIdx.x][threadIdx.y + i * COLUMNS_BLOCKDIM_Y]= (imageH - baseY > i * COLUMNS_BLOCKDIM_Y) ? d_Src[i * COLUMNS_BLOCKDIM_Y * pitch] : 0;

    //Compute and store results
    __syncthreads();
    #pragma unroll
    for(int i = COLUMNS_HALO_STEPS; i < COLUMNS_HALO_STEPS + COLUMNS_RESULT_STEPS; i++){
        float sum = 0;
        #pragma unroll
        for(int j = -k_radius; j <= k_radius; j++)
            sum += c_Kernel[k_index][k_radius - j] * s_Data[threadIdx.x][threadIdx.y + i * COLUMNS_BLOCKDIM_Y + j];

        d_Dst[i * COLUMNS_BLOCKDIM_Y * pitch] = sum;
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
    assert( COLUMNS_BLOCKDIM_Y * COLUMNS_HALO_STEPS >= k_radius );
    assert( imageW % COLUMNS_BLOCKDIM_X == 0 );
    assert( imageH % (COLUMNS_RESULT_STEPS * COLUMNS_BLOCKDIM_Y) == 0 );

    dim3 blocks(imageW / COLUMNS_BLOCKDIM_X, imageH / (COLUMNS_RESULT_STEPS * COLUMNS_BLOCKDIM_Y));
    dim3 threads(COLUMNS_BLOCKDIM_X, COLUMNS_BLOCKDIM_Y);

    convColsF32SepKernel<<<blocks, threads>>>(
        d_Dst,
        d_Src,
        imageW,
        imageH,
        imageW,
        k_Index,
        k_radius
    );
    //cutilCheckMsg("convolutionColumnsKernel() execution failed\n");
}


extern "C" void convF32Sep(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_Index,
    int k_Size,
    float* d_Buffer)
{

    if(d_Buffer == NULL)
    {
        HANDLE_ERROR( cudaMalloc((void **)&d_Buffer, imageW*imageH*sizeof(float)) );   
        convRowsF32Sep(d_Buffer, d_Src, imageW, imageH, k_Index, k_Size);
        convColsF32Sep(d_Dst, d_Buffer, imageW, imageH, k_Index, k_Size);
        HANDLE_ERROR( cudaFree(d_Buffer ) );
    }
    else
    {
        convRowsF32Sep(d_Buffer, d_Src, imageW, imageH, k_Index, k_Size);
        convColsF32Sep(d_Dst, d_Buffer, imageW, imageH, k_Index, k_Size);
    }
}


extern "C" void convF32SepAdd(
    float *d_Dst,
    float *d_Src,
    int imageW,
    int imageH,
    int k_Index,
    int k_Size,
    float* d_Buffer)
{

    if(d_Buffer == NULL)
    {
        HANDLE_ERROR( cudaMalloc((void **)&d_Buffer, imageW*imageH*sizeof(float)) );   
        convRowsF32Sep(d_Buffer, d_Src, imageW, imageH, k_Index, k_Size);
        HANDLE_ERROR( cudaDeviceSynchronize() );
        convColsF32Sep(d_Dst, d_Src, imageW, imageH, k_Index, k_Size);
        addF32(d_Dst, d_Dst, d_Buffer, imageW, imageH); 
        HANDLE_ERROR( cudaDeviceSynchronize() );
        HANDLE_ERROR( cudaFree(d_Buffer ) );
    }
    else
    {
        convRowsF32Sep(d_Buffer, d_Src, imageW, imageH, k_Index, k_Size);
        HANDLE_ERROR( cudaDeviceSynchronize() );
        convColsF32Sep(d_Dst, d_Src, imageW, imageH, k_Index, k_Size);
        HANDLE_ERROR( cudaDeviceSynchronize() );
        addF32(d_Dst, d_Dst, d_Buffer, imageW, imageH);        
        HANDLE_ERROR( cudaDeviceSynchronize() );
    }
}



/*
////////////////////////////////////////////////////////////////////////////////
// Texture convolution
////////////////////////////////////////////////////////////////////////////////
//Maps to a single instruction on G8x / G9x / G10x
#define IMAD(a, b, c) ( __mul24((a), (b)) + (c) )

//Use unrolled innermost convolution loop
//#define UNROLL_INNER 1

//Round a / b to nearest higher integer value
inline int iDivUp(int a, int b){
    return (a % b != 0) ? (a / b + 1) : (a / b);
}

//Align a to nearest higher multiple of b
inline int iAlignUp(int a, int b){
    return (a % b != 0) ?  (a - a % b + b) : a;
}

texture<float, 2, cudaReadModeElementType> texSrc;

extern "C" void setInputArray(cudaArray *a_Src){
}

extern "C" void detachInputArray(void){
}
*/

/*
// Loop unrolling templates, needed for best performance
template<int i> __device__ float convolutionRow(float x, float y){
    return 
        tex2D(texSrc, x + (float)(KERNEL_RADIUS - i), y) * c_Kernel[i]
        + convolutionRow<i - 1>(x, y);
}

template<> __device__ float convolutionRow<-1>(float x, float y){
    return 0;
}

template<int i> __device__ float convolutionColumn(float x, float y){
    return 
        tex2D(texSrc, x, y + (float)(KERNEL_RADIUS - i)) * c_Kernel[i]
        + convolutionColumn<i - 1>(x, y);
}

template<> __device__ float convolutionColumn<-1>(float x, float y){
    return 0;
}
*/

/*
// Row convolution filter
__global__ void convRowsF32TexKernel(
    float *d_Dst,
    int imageW,
    int imageH,
    int k_radius)
{
    const   int ix = IMAD(blockDim.x, blockIdx.x, threadIdx.x);
    const   int iy = IMAD(blockDim.y, blockIdx.y, threadIdx.y);
    const float  x = (float)ix + 0.5f;
    const float  y = (float)iy + 0.5f;

    if(ix >= imageW || iy >= imageH)
        return;

    float sum = 0;

    //#if(UNROLL_INNER)
    //    sum = convolutionRow<2 * k_radius>(x, y);
    //#else
        for(int k = -k_radius; k <= k_radius; k++)
        sum += tex2D(texSrc, x + (float)k, y) * c_Kernel[k_radius - k];
    //#endif

    d_Dst[IMAD(iy, imageW, ix)] = sum;
}


extern "C" void convRowsF32Tex(
    float *d_Dst,
    cudaArray *a_Src,
    int imageW,
    int imageH,
    int k_radius)
{
    dim3 threads(16, 12);
    dim3 blocks(iDivUp(imageW, threads.x), iDivUp(imageH, threads.y));

    HANDLE_ERROR( cudaBindTextureToArray(texSrc, a_Src) );
    convRowsF32TexKernel<<<blocks, threads>>>(
        d_Dst,
        imageW,
        imageH,
        k_radius
    );
    HANDLE_ERROR( cudaUnbindTexture(texSrc) );
}



////////////////////////////////////////////////////////////////////////////////
// Column convolution filter
////////////////////////////////////////////////////////////////////////////////
__global__ void convColsF32TexKernel(
    float *d_Dst,
    int imageW,
    int imageH,
    int k_radius)
{
    const   int ix = IMAD(blockDim.x, blockIdx.x, threadIdx.x);
    const   int iy = IMAD(blockDim.y, blockIdx.y, threadIdx.y);
    const float  x = (float)ix + 0.5f;
    const float  y = (float)iy + 0.5f;

    if(ix >= imageW || iy >= imageH)
        return;

    float sum = 0;

    //#if(UNROLL_INNER)
    //    sum = convolutionColumn<2 * k_radius>(x, y);
    //#else
        for(int k = -k_radius; k <= k_radius; k++)
            sum += tex2D(texSrc, x, y + (float)k) * c_Kernel[k_radius - k];
    //#endif

     d_Dst[IMAD(iy, imageW, ix)] = sum;
}


extern "C" void convColsF32Tex(
    float *d_Dst,
    cudaArray *a_Src,
    int imageW,
    int imageH,
    int k_radius)
{
    dim3 threads(16, 12);
    dim3 blocks(iDivUp(imageW, threads.x), iDivUp(imageH, threads.y));

    HANDLE_ERROR(cudaBindTextureToArray(texSrc, a_Src) );
    convColsF32TexKernel<<<blocks, threads>>>(
        d_Dst,
        imageW,
        imageH,
        k_radius);
    HANDLE_ERROR( cudaUnbindTexture(texSrc) );
}
*/

