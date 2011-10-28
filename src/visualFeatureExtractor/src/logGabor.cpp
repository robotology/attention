// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file logGabor.cpp
 * @brief Implementation of log-Gabor filters. This is useful in implementing phase congruency methods (eg. one by Peter
 *  Kivoski).
 */

#include<iCub/logGabor.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
logGabor::logGabor(){
    
    // Allocating space for non radix 2 FFT. 2D FFT are implemented as FFT_of_Col(FFT_of_Rows)
    wt_row = gsl_fft_complex_wavetable_alloc(ROW_SIZE);
    wk_row = gsl_fft_complex_workspace_alloc(ROW_SIZE);
    wt_col = gsl_fft_complex_wavetable_alloc(COL_SIZE);
    wk_col = gsl_fft_complex_workspace_alloc(COL_SIZE);

    // Allocations for storing log Gabor filters and other variables
    logGaborFilter  = new logGaborOrientArray[LOG_GABOR_ORIENTATION];    // stores LG for all orienatation and scales
    imgInComplex    = new logGaborComplexRow[COL_SIZE];                  // to store image in complex notation    

    

}

logGabor::~logGabor(){

    gsl_fft_complex_wavetable_free (wt_row);
    gsl_fft_complex_workspace_free (wk_row);
    gsl_fft_complex_wavetable_free (wt_col);
    gsl_fft_complex_workspace_free (wk_col);

    delete [] logGaborFilter;
    delete [] imgInComplex;
    
    

}

void logGabor::FFT2D(double input2DArray[COL_SIZE][2*ROW_SIZE], double FFTed[COL_SIZE][2*ROW_SIZE], bool forward){

    //const int h = COL_SIZE;
    //const int w = ROW_SIZE;
    //double FFTed[h][2*w];
    // Calculate FFT of each row
    for(int i= 0; i < COL_SIZE; ++i){
        // get a row and convert to complex
        double currentRow[2*ROW_SIZE];
        for(int j = 0; j < 2*ROW_SIZE; ++j){
            //currentRow[2*j]=input2DRealArray[i][j];
            //currentRow[2*j+1]=0;
            currentRow[j] = input2DArray[i][j];
        }
        // Calculate FFT of this row
        if(forward)
            gsl_fft_complex_forward(currentRow, 1, ROW_SIZE,wt_row,wk_row);
        else
            gsl_fft_complex_inverse (currentRow, 1, ROW_SIZE,wt_row,wk_row);
        // store this
        for(int j = 0; j < 2*ROW_SIZE; ++j){
            FFTed[i][j]=currentRow[j];
        }  
    }

    // Calculate FFT of each column
    for(int i=0; i<ROW_SIZE; ++i){
        // get a column
        double currentCol[2*COL_SIZE];
        for(int j=0; j<COL_SIZE; ++j){
            currentCol[2*j]=FFTed[j][2*i];
            currentCol[2*j+1]=FFTed[j][2*i+1];
        }
        // Calculate FFT of this column
        if(forward)
            gsl_fft_complex_forward (currentCol, 1, COL_SIZE,wt_col,wk_col);
        else
            gsl_fft_complex_inverse (currentCol, 1, COL_SIZE,wt_col,wk_col);
        // store this
        for(int j = 0; j < COL_SIZE; ++j){
            FFTed[j][2*i]=currentCol[2*j];
            FFTed[j][2*i+1]=currentCol[2*j+1];
        }
    }

    // FFTed now has 2D FFT of input2DArray by column wise FFT of row-wise FFT of input2DArray 
  
}

// Overloading FFT calculation method, input is real image for forward direction optimization
void logGabor::FFT2D(ImageOf<PixelMono>*  inputImage, double FFTed[COL_SIZE][2*ROW_SIZE], bool forward){

    //const int h = COL_SIZE;
    //const int w = ROW_SIZE;
    //double FFTed[h][2*w];
    // Calculate FFT of each row
    int pad = inputImage->getPadding();
    uchar* ptrInptImg =(uchar*)inputImage->getRawImage(); 
    // Calculate FFT of each row
    for(int i= 0; i < COL_SIZE; ++i){
        // get a row and convert to complex
        double currentRow[2*ROW_SIZE];
        for(int j = 0; j < 2*ROW_SIZE; ++j){
            //currentRow[2*j]=input2DRealArray[i][j];
            //currentRow[2*j+1]=0;
            currentRow[j] = *ptrInptImg++;//input2DArray[i][j];
        }
        // Calculate FFT of this row
        if(forward)
            gsl_fft_complex_forward(currentRow, 1, ROW_SIZE,wt_row,wk_row);
        else
            gsl_fft_complex_inverse (currentRow, 1, ROW_SIZE,wt_row,wk_row);
        // store this
        for(int j = 0; j < 2*ROW_SIZE; ++j){
            FFTed[i][j]=currentRow[j];
        }
        // shift for padding
        ptrInptImg += pad;  
    }
    

    // Calculate FFT of each column
    for(int i=0; i<ROW_SIZE; ++i){
        // get a column
        double currentCol[2*COL_SIZE];
        for(int j=0; j<COL_SIZE; ++j){
            currentCol[2*j]=FFTed[j][2*i];
            currentCol[2*j+1]=FFTed[j][2*i+1];
        }
        // Calculate FFT of this column
        if(forward)
            gsl_fft_complex_forward (currentCol, 1, COL_SIZE,wt_col,wk_col);
        else
            gsl_fft_complex_inverse (currentCol, 1, COL_SIZE,wt_col,wk_col);
        // store this
        for(int j = 0; j < COL_SIZE; ++j){
            FFTed[j][2*i]=currentCol[2*j];
            FFTed[j][2*i+1]=currentCol[2*j+1];
        }
    }

    // FFTed now has 2D FFT of input2DArray by column wise FFT of row-wise FFT of input2DArray 
  
}

// Overloading FFT calculation method, input is real image for forward direction optimization
void logGabor::FFT2D(double input2DArray[COL_SIZE][2*ROW_SIZE], ImageOf<PixelFloat>* FFTRealImage,ImageOf<PixelFloat>* FFTCplxImage, bool forward){

    //const int h = COL_SIZE;
    //const int w = ROW_SIZE;
    //double FFTed[h][2*w];
    // Calculate FFT of each row
    int padR = FFTRealImage->getPadding();
    int padC = FFTCplxImage->getPadding();
    uchar* ptrInptImg =(uchar*)inputImage->getRawImage(); 
    // Calculate FFT of each row
    for(int i= 0; i < COL_SIZE; ++i){
        // get a row and convert to complex
        double currentRow[2*ROW_SIZE];
        for(int j = 0; j < 2*ROW_SIZE; ++j){
            //currentRow[2*j]=input2DRealArray[i][j];
            //currentRow[2*j+1]=0;
            currentRow[j] = input2DArray[i][j];
        }
        // Calculate FFT of this row
        if(forward)
            gsl_fft_complex_forward(currentRow, 1, ROW_SIZE,wt_row,wk_row);
        else
            gsl_fft_complex_inverse (currentRow, 1, ROW_SIZE,wt_row,wk_row);
        // store this
        for(int j = 0; j < 2*ROW_SIZE; ++j){
            FFTed[i][j]=currentRow[j];
        }
         
    }
    

    // Calculate FFT of each column
    for(int i=0; i<ROW_SIZE; ++i){
        // get a column
        double currentCol[2*COL_SIZE];
        for(int j=0; j<COL_SIZE; ++j){
            currentCol[2*j]=FFTed[j][2*i];
            currentCol[2*j+1]=FFTed[j][2*i+1];
        }
        // Calculate FFT of this column
        if(forward)
            gsl_fft_complex_forward (currentCol, 1, COL_SIZE,wt_col,wk_col);
        else
            gsl_fft_complex_inverse (currentCol, 1, COL_SIZE,wt_col,wk_col);
        // store this
        for(int j = 0; j < COL_SIZE; ++j){
            FFTed[j][2*i]=currentCol[2*j];
            FFTed[j][2*i+1]=currentCol[2*j+1];
        }
    }

    // FFTed now has 2D FFT of input2DArray by column wise FFT of row-wise FFT of input2DArray 
  
}

void logGabor::setLogGabor(int m, double sigmaF, double minWave, double cuttoff_butterworth, int sharpness_butterworth){

    /* Setting up log-Gabor filter */   
       
    
    double radius[COL_SIZE][ROW_SIZE];
    double theta[COL_SIZE][ROW_SIZE];   

    double freq0;

    double maxWave = minWave*pow(m,(LOG_GABOR_SCALE-1));
    

    double spread;

    // shift the radius
    int row, col;
    double  x, y;

    int orientation = 0; double waveLength = minWave;
    double ang ;//= (orientation)*PI_Gab / (LOG_GABOR_ORIENTATION);
    double deltaTheta;

    // get FFT of logGabor filter for each scale and each orientation

    
    for(int scaleOfFilter = 0;scaleOfFilter<LOG_GABOR_SCALE; ++scaleOfFilter){ //for each scale
    
        for(int orientOfFilter =0;orientOfFilter<LOG_GABOR_ORIENTATION; ++orientOfFilter){  //for each orientation

            
            freq0 = 1.0/(minWave*pow(m,scaleOfFilter));
            ang = orientOfFilter*PI_Gab/LOG_GABOR_ORIENTATION;
         
            for(int i=0; i<COL_SIZE; ++i){

                for(int j=0; j<ROW_SIZE; ++j){

                    row = i<COL_SIZE/2?i+COL_SIZE/2:i-COL_SIZE/2;     // Quadrant shift, FFT reasons
                    col = j<ROW_SIZE/2?j+ROW_SIZE/2:j-ROW_SIZE/2;
                    x = -.5 + (double)i/(COL_SIZE-1);     // assuming COL_SIZE is even
                    y = -.5 + (double)j/(ROW_SIZE-1);     // assuming ROW_SIZE is even
                    radius[row][col]= sqrt(x*x + y*y)/freq0;
                    theta[row][col]= atan2(y,x);

                    if(radius[row][col] < .001){     // since log is singular near zero
                        logGaborFilter[orientOfFilter][scaleOfFilter][i][j] = 0;
                    }
                    else{
                        logGaborFilter[orientOfFilter][scaleOfFilter][i][j] = 
                                                               exp(-pow(log(radius[row][col]),2)/(2*pow(log(sigmaF),2)));

                        // Apply butterworth lowpass filter
                        logGaborFilter[orientOfFilter][scaleOfFilter][i][j] = logGaborFilter[orientOfFilter][scaleOfFilter][i][j]/(1.0+ pow((radius[row][col]/cuttoff_butterworth),2*sharpness_butterworth));
                    }

                    deltaTheta = abs(atan2(sin(theta[row][col])*cos(ang) - cos(theta[row][col])* sin(ang), cos(theta[row][col])*cos(ang) + sin(theta[row][col])*sin(ang)));

                    deltaTheta = min(PI_Gab, deltaTheta*LOG_GABOR_ORIENTATION/2);

                    logGaborFilter[orientOfFilter][scaleOfFilter][i][j] *= (cos(deltaTheta)+1)/2.0;
                                     
                    
                 }
            }
        
        
        } // end each orientation

    } //end each scale

    logGaborReady = true;

}

void logGabor::applyGaussianOnLogGabor(float sigmaX, float meanX,float sigmaY, float meanY, float scale, int ori, int scl, int centerX, int centerY){
    logGaborComplexArray* gaussNinverse = new logGaborComplexArray[2];
    // set Gaussian to this
    for(int i=0; i<COL_SIZE; ++i){
        for(int j=0; j<ROW_SIZE; ++j){

            double x = (double)abs(centerX-j);
            double y = (double)abs(centerY-i);
            gaussNinverse[0][i][2*j]= scale * exp(-.5*pow((x-meanX)/sigmaX,2) - .5*pow((y-meanY)/sigmaY,2));
            gaussNinverse[0][i][2*j+1]= scale * exp(-.5*pow((x-meanX)/sigmaX,2) - .5*pow((y-meanY)/sigmaY,2));
        }
    }

    FFT2D(gaussNinverse[0],gaussNinverse[1],true);

    // Apply the iFFT to the Log-Gabor
    int scaleOfFilter, countScl, orientOfFilter, countOri;
    if(scl<0){
        countScl = LOG_GABOR_SCALE;
        scaleOfFilter = 0;
    }
    else {
        countScl = scl+1;
        scaleOfFilter = scl;
    }
    if(ori<0){
        countOri = LOG_GABOR_ORIENTATION;
        orientOfFilter = 0;
    }
    else {
        countOri = ori+1;
        orientOfFilter = ori;
    }


    for(;scaleOfFilter<countScl; ++scaleOfFilter){ //for each scale    
        for(;orientOfFilter<countOri; ++orientOfFilter){  //for each orientation
            for(int i=0; i<COL_SIZE; ++i){
                for(int j=0; j<ROW_SIZE; ++j){
                    logGaborFilter[orientOfFilter][scaleOfFilter][i][j] *= gaussNinverse[1][i][2*j]; // only real part?
                }
            }
        }
    } 
            

    
}

void logGabor::getOneLogGabor(ImageOf<PixelMono>* inputImage,ImageOf<PixelFloat>* outputRealImage,ImageOf<PixelFloat>* outputImgImage, int scale, int orient){

    if(!logGaborReady) return; // set the Log Gabor filters first

    double imgInComplex[COL_SIZE][ROW_SIZE*2];
    double FFTofImage[COL_SIZE][ROW_SIZE*2];
    double iFFTofImage[COL_SIZE][ROW_SIZE*2];

    uchar* imgNow= (uchar*)inputImage->getRawImage();
    uchar* imgOrig = (uchar*)inputImage->getRawImage();

    // get FFT format of image
    for(int i=0; i<inputImage->height();++i){
        imgNow = imgOrig + i*inputImage->getRowSize();
        for(int j=0; j<inputImage->width();++j){

        imgInComplex[i][2*j] = (double)*imgNow;
        imgInComplex[i][2*j+1] = (double)*imgNow++;
        }
    }

    FFT2D(imgInComplex,FFTofImage,true); // forward

    // get element wise product of these two matrices

    for(int i= 0; i<COL_SIZE; ++i){
        for(int j= 0; j<ROW_SIZE; ++j){
            FFTofImage[i][2*j] *=logGaborFilter[orient][scale][i][j];  // first scale first orientation
            FFTofImage[i][2*j+1] *= logGaborFilter[orient][scale][i][j]; // may not work
        }
    }
    
    FFT2D(FFTofImage,iFFTofImage,false);    // inverse

    // Store the result
    float* outReal = (float*)outputRealImage->getRawImage();
    float* outImg = (float*)outputImgImage->getRawImage();
    float* imgNowR;
    float* imgNowC;
    for(int i=0; i<outputRealImage->height();++i){
        imgNowR = outReal + i*outputRealImage->getRowSize();
        imgNowC = outImg + i*outputImgImage->getRowSize();
        for(int j=0; j<outputRealImage->width();++j){
            *imgNowR++ = iFFTofImage[i][2*j] ;
            *imgNowC++ = iFFTofImage[i][2*j+1] ;        
        }
    }

}

    
void logGabor::getLogGaborForOrient(ImageOf<PixelMono>* inputImage,ImageOf<PixelFloat>* outputRealImage,ImageOf<PixelFloat>* outputImgImage,double weightage[LOG_GABOR_SCALE],int orient){

    if(!logGaborReady) return; // set the Log Gabor filters first

    double wt[LOG_GABOR_SCALE];
    if(weightage == NULL){
        for(int i=0; i<LOG_GABOR_SCALE; ++i)
            wt[i]= 1.0/LOG_GABOR_SCALE;
    }
    else{
        for(int i=0; i<LOG_GABOR_SCALE; ++i)
            wt[i]= weightage[i];
    }


    double imgInComplex[COL_SIZE][ROW_SIZE*2];
    double FFTofImage[COL_SIZE][ROW_SIZE*2];
    double iFFTofImage[COL_SIZE][ROW_SIZE*2];

    uchar* imgNow= (uchar*)inputImage->getRawImage();
    uchar* imgOrig = (uchar*)inputImage->getRawImage();

    // get FFT format of image
    for(int i=0; i<inputImage->height();++i){
        imgNow = imgOrig + i*inputImage->getRowSize();
        for(int j=0; j<inputImage->width();++j){

        imgInComplex[i][2*j] = (double)*imgNow;
        imgInComplex[i][2*j+1] = (double)*imgNow++;
        }
    }

    FFT2D(imgInComplex,FFTofImage,true); // forward

    for(int eachScale=0;eachScale<LOG_GABOR_SCALE; ++eachScale){

        // get element wise product of these two matrices

        for(int i= 0; i<COL_SIZE; ++i){
            for(int j= 0; j<ROW_SIZE; ++j){
                FFTofImage[i][2*j] *=logGaborFilter[orient][eachScale][i][j];  // first scale first orientation
                FFTofImage[i][2*j+1] *= logGaborFilter[orient][eachScale][i][j]; // may not work
            }
        }
        
        FFT2D(FFTofImage,iFFTofImage,false);    // inverse

        // Accumulate the result
        float* outReal = (float*)outputRealImage->getRawImage();
        float* outImg = (float*)outputImgImage->getRawImage();
        float* imgNowR;
        float* imgNowC;
        for(int i=0; i<outputRealImage->height();++i){
            imgNowR = outReal + i*outputRealImage->getRowSize();
            imgNowC = outImg + i*outputImgImage->getRowSize();
            for(int j=0; j<outputRealImage->width();++j){
                *imgNowR++ += wt[eachScale]*iFFTofImage[i][2*j] ;
                *imgNowC++ += wt[eachScale]*iFFTofImage[i][2*j+1] ;        
            }
        }
    } // end of each scales

}

void logGabor::getAllLogGabor(ImageOf<PixelMono>* inputImage,ImageOf<PixelFloat>* outputRealImage,ImageOf<PixelFloat>* outputImgImage,double weightScale[LOG_GABOR_SCALE],double weightOrient[LOG_GABOR_ORIENTATION]){

    if(!logGaborReady) return; // set the Log Gabor filters first

    double wtS[LOG_GABOR_SCALE];
    if(weightScale == NULL){
        for(int i=0; i<LOG_GABOR_SCALE; ++i)
            wtS[i]= 1.0/LOG_GABOR_SCALE;
    }
    else{
        for(int i=0; i<LOG_GABOR_SCALE; ++i)
            wtS[i]= weightScale[i];
    }

    double wtO[LOG_GABOR_ORIENTATION];
    if(weightOrient == NULL){
        for(int i=0; i<LOG_GABOR_ORIENTATION; ++i)
            wtO[i]= 1.0/LOG_GABOR_ORIENTATION;
    }
    else{
        for(int i=0; i<LOG_GABOR_ORIENTATION; ++i)
            wtO[i]= weightOrient[i];
    }


    double imgInComplex[COL_SIZE][ROW_SIZE*2];
    double FFTofImage[COL_SIZE][ROW_SIZE*2];
    double iFFTofImage[COL_SIZE][ROW_SIZE*2];

    uchar* imgNow= (uchar*)inputImage->getRawImage();
    uchar* imgOrig = (uchar*)inputImage->getRawImage();

    // get FFT format of image
    for(int i=0; i<inputImage->height();++i){
        imgNow = imgOrig + i*inputImage->getRowSize();
        for(int j=0; j<inputImage->width();++j){

        imgInComplex[i][2*j] = (double)*imgNow;
        imgInComplex[i][2*j+1] = (double)*imgNow++;
        }
    }

    FFT2D(imgInComplex,FFTofImage,true); // forward

    for(int eachOrient=0; eachOrient<LOG_GABOR_ORIENTATION; ++eachOrient){

        for(int eachScale=0;eachScale<LOG_GABOR_SCALE; ++eachScale){

            // get element wise product of these two matrices

            for(int i= 0; i<COL_SIZE; ++i){
                for(int j= 0; j<ROW_SIZE; ++j){
                    FFTofImage[i][2*j] *=logGaborFilter[eachOrient][eachScale][i][j];  // first scale first orientation
                    FFTofImage[i][2*j+1] *=logGaborFilter[eachOrient][eachScale][i][j]; // may not work
                }
            }
            
            FFT2D(FFTofImage,iFFTofImage,false);    // inverse

            // Accumulate the result
            float* outReal = (float*)outputRealImage->getRawImage();
            float* outImg = (float*)outputImgImage->getRawImage();
            float* imgNowR;
            float* imgNowC;
            float wtFactor = wtS[eachScale]*wtO[eachOrient];
            for(int i=0; i<outputRealImage->height();++i){
                imgNowR = outReal + i*outputRealImage->getRowSize()/sizeof(float);
                imgNowC = outImg + i*outputImgImage->getRowSize()/sizeof(float);
                for(int j=0; j<outputRealImage->width();++j){
                    *imgNowR++ += wtFactor*iFFTofImage[i][2*j] ;
                    *imgNowC++ += wtFactor*iFFTofImage[i][2*j+1] ;        
                }
            }
        } // end of each scales
    }// end of each orientation

}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

