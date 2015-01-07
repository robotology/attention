// -*- mode:C++; tab-width():4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Shashank Pathak
 * email:   francesco.rea@iit.it, shashank.pathak@iit.it
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
 * @file convolve.h
 * @brief Simple, efficient and general method to compute convolution
 */

#ifndef _CONVOLVE_H_
#define _CONVOLVE_H_

template<class inputImage,class ptrInput, class outputImage, class ptrOutput>
class convolve {

        int kernelWidth;        // width of the kernel. When 1D kernel is applied in horizontal, this is length
        int kernelHeight;       // height of the kernel. When 1D kernel is applied in vertical, this is length       
        float* kernel;          // pointer to kernel values
        int nbrOfKernels;       // number of kernels in the list of kernels, default 0
        float** listOfKernels;  // pointer to pointers to many kernels
        int direction;          // direction in which kernel is applied,(0,1,2) for (horizontal,vertical,both)
        float factor;           // scaling factor applied to kernel multiplication
        int shift;              // shift in values applied to kernel multiplication
        bool kernelIsDefined;   // flag to check that kernel values are set properly
        int counter;            // this counts number of times convolution is applied, till it reaches flicker threshold
        int flicker;            // threshold after which convolution is static
        float limits[2];        // minimum and maximum values attained after counter equals threshold

        // Not allowing copying and assignment
        void operator=(const convolve&);
        convolve(const convolve&);

        inline int _max(int a, int b) {return a<b?b:a;}

    public:
        convolve(){
            this->kernelWidth   = 0;
            this->kernelHeight  = 0;
            this->kernel        = NULL;
            this->listOfKernels = NULL;
            this->nbrOfKernels  = 0;
            this->direction     = -1;
            this->factor        = 0;
            this->shift         = 0;
            this->kernelIsDefined = false;
            this->counter       = 0;
            this->flicker       = -1;
            this->limits        = NULL; 
        };

    /**
     * For a two dimensional kernel, for inseparable kernels 
     * @param width width of kernel
     * @param height height of kernel
     * @param kernel pointer to float array representing kernel values
     * @param scale scaling factor applied to kernel multiplication
     * @param shift shift in values applied to kernel multiplication
     * @param flicker count of kernel operations where the min and max will be calculated to normalise it later
     */
        convolve(int width,int height,float* kernel, float scale,int shift,int flicker=0){
        
            this->kernelWidth   = width;
            this->kernelHeight  = height;
            this->kernel        = kernel;
            this->listOfKernels = NULL;
            this->nbrOfKernels  = 0;
            this->direction     = 2;
            this->factor        = scale;
            this->shift         = shift;
            this->counter       = 0;
            this->flicker       = flicker;
            this->limits[0]     = -10000;       // max
            this->limits[1]     = 10000;        // min
            if(kernel == NULL || width <0 || height<0){
                this->kernelIsDefined = false;
            }
            else {
                this->kernelIsDefined = true;               
            }
            assert(this->kernelIsDefined);
        };
    /**
     * For a linear kernel. For separable 2D kernels, this can be efficient way to do
     * @param length legnth of vector representing values of kernel
     * @param kernel pointer to float array representing kernel values
     * @param direction direction of the kernel, 0 implies horizontal and 1 implies vertical
     * @param scale scaling factor applied to kernel multiplication
     * @param shift shift in values applied to kernel multiplication
     * @param flicker count of kernel operations where the min and max will be calculated to normalise it later
     */
        convolve(int length,float* kernel,int direction,float scale,int shift,int flicker=0){
        
            if(direction == 0) {
                this->kernelHeight = 0;
                this->kernelWidth   = length;                
            }
            else if(direction == 1){
                this->kernelHeight = length;
                this->kernelWidth   = 0;                
            }
            this->kernel        = kernel;
            this->listOfKernels = NULL;
            this->nbrOfKernels  = 0;
            this->direction     = direction;
            this->factor        = scale;
            this->shift         = shift;
            this->counter       = 0;
            this->flicker       = flicker;
            if(flicker>0){
                this->limits[0]     = -10000;       // max
                this->limits[1]     = 10000;        // min
            }
            else {
                this->limits[0] = 255.0;
                this->limits[1] = 0.0;
            }
            if(kernel == NULL || length <0){
                this->kernelIsDefined = false;
            }
            else {
                this->kernelIsDefined = true;               
            }
            assert(this->kernelIsDefined);
        };
        ~convolve(){
            //nothing 
        };
    /**
     * For a list of inseparable 2D kernels 
     * @param width width of kernel
     * @param height height of kernel
     * @param kernel pointer to float array representing kernel values
     * @param scale scaling factor applied to kernel multiplication
     * @param shift shift in values applied to kernel multiplication
     * @param flicker count of kernel operations where the min and max will be calculated to normalise it later
     */
        convolve(int width,int height,float** kernelList, int kernelNbrs, float scale=1,int shift=0,int flicker=0){
        
            this->kernelWidth   = width;
            this->kernelHeight  = height;
            this->kernel        = NULL;
            this->listOfKernels = kernelList;
            this->nbrOfKernels  = kernelNbrs;
            this->direction     = 2;
            this->factor        = scale;
            this->shift         = shift;
            this->counter       = 0;
            this->flicker       = flicker;
            this->limits[0]     = -10000;       // max
            this->limits[1]     = 10000;        // min
            if(listOfKernels == NULL || nbrOfKernels < 1 || width <0 || height<0){
                this->kernelIsDefined = false;
            }
            else {
                this->kernelIsDefined = true;               
            }
            assert(this->kernelIsDefined);
        };

    /**
     * For setting a kernel. This could be used to reuse a kernel object
     * @param length legnth of vector representing values of kernel
     * @param kernel pointer to float array representing kernel values
     * @param direction direction of the kernel, 0 implies horizontal and 1 implies vertical
     * @param scale scaling factor applied to kernel multiplication
     * @param shift shift in values applied to kernel multiplication
     * @param flicker count of kernel operations where the min and max will be calculated to normalise it later
     * @param upLimit the upper limit observed in kernel output during convolution(used for normalising)
     * @param downLimit the lower limit observed in kernel output during convolution(used for normalising)
     */

        void setKernelParameters(int width,int height,float* kernel, float** kernelList, int kernelNbrs, float scale,int shift,int flicker, float upLimit, float downLimit){
        
            this->kernelWidth   = width;
            this->kernelHeight  = height;
            this->kernel        = kernel;
            this->listOfKernels = kernelList;
            this->nbrOfKernels  = kernelNbrs;            
            this->direction     = 2;
            this->factor        = scale;
            this->shift         = shift;
            this->counter       = 0;
            this->flicker       = flicker;
            this->limits[0]     = upLimit;       // max
            this->limits[1]     = downLimit;        // min
            this->kernelIsDefined = true;               //LATER: more assertions
        };


    /**
     * For 1D convolution ie convolving a vector with a matrix
     * @param img input image
     * @param resImg resultant image after applying the kernel
     * @param borderType an integer parameter for type of border 0: kernel from (0,0) 1: kernel all within
     */
        void convolve1D(inputImage* img,outputImage* resImg, int borderType = 0){
            assert(kernelIsDefined);
            int rowSize         = img->getRowSize()/sizeof(ptrInput);
            int resRowSize = resImg->getRowSize()/sizeof(ptrOutput);
            ptrInput* mat = (ptrInput*)img->getRawImage();
            ptrOutput* res = (ptrOutput*)resImg->getRawImage();
            int rowPos = 0; int pixPos =0;    
            float scalingVal = 1.0/(limits[0] -limits[1]);
            int shiftSqKernel = 0;
            if(borderType == 1){
                shiftSqKernel = kernelWidth/2;
            }
            if(this->direction == 0){ //horizontal convolution
                float* midVec = kernel + this->kernelWidth/2; // middle of linear kernel    
                for(int i=shiftSqKernel;i<resImg->height()-shiftSqKernel;++i){
                    for(int j=shiftSqKernel;j<resImg->width()-shiftSqKernel;++j){
                        rowPos = i*resRowSize;
                        pixPos = j;
                        float sum = *midVec * *(mat+rowPos+pixPos);
                        pixPos--;
                        for(int k=0; k<this->kernelWidth/2 && pixPos>0; k++, pixPos--){
                            sum += (*(mat+rowPos+pixPos))* (*(midVec-k));
                            
                        }
                        pixPos = j+1;
                        for(int k=0; k<this->kernelWidth/2 && pixPos<img->width(); k++, pixPos++){
                            sum += (*(mat+rowPos+pixPos))* (*(midVec+k));
                            
                        }
                        sum *= factor;
                        if(this->counter<this->flicker){
                                this->limits[0] = this->limits[0]<sum?sum:this->limits[0];
                                this->limits[1] = this->limits[1]>sum?sum:this->limits[1];
                                *(res+i*resRowSize+j)=sum;
                                this->counter++;
                                   
                        } 
                        *(res+i*resRowSize+j)=sum;

                    }
                } 
            } 
            else if(this->direction == 1){
                float* midVec = kernel + this->kernelHeight/2; // middle of linear kernel    
                for(int i=shiftSqKernel;i<resImg->height()-shiftSqKernel;++i){
                    for(int j=shiftSqKernel;j<resImg->width()-shiftSqKernel;++j){
                        rowPos = j;
                        pixPos = i;
                        float sum = *midVec * *(mat+pixPos*rowSize+rowPos);
                        pixPos--;
                        for(int k=0; k<this->kernelHeight/2 && pixPos>0; k++, pixPos--){
                            sum += (*(mat+rowPos+pixPos*rowSize))* (*(midVec-k));
                            //tmpVec++;
                        }
                        pixPos = i+1;
                        for(int k=0; k<this->kernelHeight/2 && pixPos<img->height(); k++, pixPos++){

                            sum += (*(mat+rowPos+pixPos*rowSize))* (*(midVec+k));
                            
                        }
                        sum *= factor;
                        if(this->counter<this->flicker){
                                this->limits[0] = this->limits[0]<sum?sum:this->limits[0];
                                this->limits[1] = this->limits[1]>sum?sum:this->limits[1];
                                *(res+i*resRowSize+j)=sum;
                                this->counter++;
                                   
                        } 
                        *(res+i*resRowSize+j)=sum;
                    }
                } 
            }
           else {
                assert(direction==0 || direction==1); // wrong direction
                }
            
        }

    /**
     * For 2D convolution ie convolving a matrix with a matrix. This is unavoidable when kernel is non-seperable
     * @param img input image
     * @param resImg resultant image after applying the kernel
     * @param borderType an integer parameter for type of border 0: kernel from (0,0) 1: kernel all within
     */
        void convolve2D(inputImage* img,outputImage* resImg, int borderType = 0){
            printf("convolving 2d %f %d \n",factor,shift);
            assert(kernelIsDefined);
            int rowSize    = img->getRowSize()    / sizeof(ptrInput);
            int resRowSize = resImg->getRowSize() / sizeof(ptrOutput);
            
            ptrInput*  mat          = (ptrInput*) img->getRawImage();
            ptrOutput* res          = (ptrOutput*)resImg->getRawImage();
            ptrInput*  currPtrImage = (ptrInput*) img->getRawImage();

            int rowPos = 0; 
            int pixPos = 0;            
            int padOutput     = resImg->getPadding()/sizeof(ptrOutput);
            float* kerStartPt = this->kernel; 
            float scalingVal  = 1.0/(limits[0] -limits[1]);           
            int shiftSqKernel = 0;

            if(borderType == 1){
                shiftSqKernel = kernelWidth/2;
            }

            for(int i = shiftSqKernel ; i < resImg->height() - shiftSqKernel ; ++i){
                int eff_ht = min(img->height(),i+kernelHeight/2)-_max(0,i-kernelHeight/2)+1;
                for(int j  = shiftSqKernel ; j < resImg->width() - shiftSqKernel ; ++j){
                    // current pixel point is anchor
                    int eff_wd = min(img->width(), j + kernelWidth/2)- _max(0,j-kernelWidth/2)+1;
                    currPtrImage = mat    + rowSize * _max(0,i-kernelHeight/2)+_max(0,j-kernelWidth/2);
                    kerStartPt   = kernel + _max(0,kernelHeight/2 -i)*kernelWidth + _max(0,kernelWidth/2-j);
                    float sum = 0;
                    for(int k = 0 ; k < eff_ht ; ++k){
                        for(int l = 0 ; l < eff_wd ; ++l){
                           sum += *currPtrImage++ * *kerStartPt++ * factor;
                        }
                        // shift the pointers
                        currPtrImage += rowSize - eff_wd-1;
                        kerStartPt   += _max(0,j + kernelWidth / 2 - img->width());
                    }
                    if(this->counter<this->flicker){
                        this->limits[0] = this->limits[0] < sum ? sum : this->limits[0];
                        this->limits[1] = this->limits[1] > sum ? sum : this->limits[1];
                        this->counter++;
                        
                    } 
                    *res++ = sum;                    
                }
                res += padOutput;
            }
        }

    /**
     * For list of 2D convolution ie convolving a list of matrix with a matrix. This is unavoidable when kernel 
     *  is non-seperable
     * @param img input image
     * @param resImg list of resultant image after applying the kernels respectively
     * @param borderType an integer parameter for type of border 0: kernel from (0,0) 1: kernel all within
     */
    void convolve2Dlist(inputImage* img,outputImage** resImg, int borderType = 0){
            
            assert(kernelIsDefined);
            int rowSize    = img->getRowSize()       / sizeof(ptrInput);
            int resRowSize = resImg[0]->getRowSize() / sizeof(ptrOutput);
            const int nbr_kernels   = this->nbrOfKernels;
            const int width_kernel  = this->kernelWidth;
            const int height_kernel = this->kernelHeight;
            float* kerStartPt[nbr_kernels],*kerOrigins[nbr_kernels];

            for(int i=0;i<nbr_kernels;++i){ 
                    kerStartPt[i] = *(this->listOfKernels+i);
                    kerOrigins[i] = *(this->listOfKernels+i);
            }

            int padOutput[nbr_kernels];
            ptrOutput* res[nbr_kernels];
            float sum[nbr_kernels];
            
            ptrInput* mat          = (ptrInput*)img->getRawImage();
            ptrInput* currPtrImage = (ptrInput*)img->getRawImage();
            int rowPos = 0; int pixPos =0;
            for(int cntK=0;cntK<nbr_kernels;cntK++){
                    res[cntK] = (ptrOutput*)resImg[cntK]->getRawImage();            
                    padOutput[cntK] = resImg[cntK]->getPadding()/sizeof(ptrOutput);
                    //kerStartPt[cntK] = (this->listOfKernels)[cntK];
            }
 
            float scalingVal = 1.0/(limits[0] -limits[1]);           
            int shiftSqKernel = 0;
            if(borderType == 1){
                shiftSqKernel = kernelWidth/2;
            }
            for(int i=shiftSqKernel;i<img->height()-shiftSqKernel;++i){
                int eff_ht = min(img->height(),i+kernelHeight/2)-_max(0,i-kernelHeight/2)+1;
                for(int j=shiftSqKernel;j<img->width()-shiftSqKernel;++j){
                    // current pixel point is anchor
                    int eff_wd = min(img->width(), j + kernelWidth/2)- _max(0,j-kernelWidth/2)+1;
                    currPtrImage = mat + rowSize*_max(0,i-kernelHeight/2)+_max(0,j-kernelWidth/2);
                    for(int p=0;p<nbr_kernels;++p){
                        kerStartPt[p] = kerOrigins[p] + _max(0,kernelHeight/2 -i)*kernelWidth + _max(0,kernelWidth/2-j);
                        sum[p]=0;
                    }
                     
                    for(int k=0; k<eff_ht;++k){
                        for(int l=0;l<eff_wd;++l){
                                for(int p=0;p<nbr_kernels;++p){
                                        sum[p] += *currPtrImage * *kerStartPt[p]++*factor;
                                }
                                currPtrImage++;
                        }
                        // shift the pointers
                        currPtrImage += rowSize - eff_wd-1;
                        for(int p=0;p<nbr_kernels;++p){
                                kerStartPt[p] += _max(0,j+kernelWidth/2-img->width());
                        }
                    }
                   for(int p=0;p<nbr_kernels;++p){
                        *res[p]++ = sum[p];
                   }               
                    
                }
                for(int p=0;p<nbr_kernels;++p){
                        res[p] += padOutput[p];
                }
            }

                    
   }

   /**
     * For 2D convolution of a specific region (for now circle). This is to save some computational efforts
     * @param img input image
     * @param resImg resultant image after applying the kernel
     * @param regionType an integer parameter for type of region 0: circle
     * @param firstDimension an integer for first dimension (is radius for circle)
     * @param secondDimension an integer for second dimension (is same as first for circle/square)
     * @param centerX an integer for X coordinate of center
     * @param centerY an integer for Y coordinate of center
     * @param borderType an integer parameter for type of border 0: kernel from (0,0) 1: kernel all within
     */
        void convolve2DRegion(inputImage* img,outputImage* resImg, int regionType, int firstDimension ,int secondDimension,int centerX, int centerY, int borderType = 0){
            
            assert(kernelIsDefined);
            if(regionType == 0){
                secondDimension = firstDimension;                
            }
            float radiusSquared = firstDimension*firstDimension;
            float radiusIntoRootTwo = sqrt(2.0)*firstDimension;
            int rowSize         = img->getRowSize()/sizeof(ptrInput);
            int resRowSize = resImg->getRowSize()/sizeof(ptrOutput);
            
            ptrInput* mat = (ptrInput*)img->getRawImage();
            ptrOutput* res = (ptrOutput*)resImg->getRawImage();
            int rowPos = 0; int pixPos =0;
            ptrInput* currPtrImage = (ptrInput*)img->getRawImage();
            int padOutput = resImg->getPadding()/sizeof(ptrOutput);
            float* kerStartPt = this->kernel; 
            float scalingVal = 1.0/(limits[0] -limits[1]);           
            int shiftSqKernel = 0;
            if(borderType == 1){
                shiftSqKernel = kernelWidth/2;
            }
            for(int i=shiftSqKernel;i<resImg->height()-shiftSqKernel;++i){
                int eff_ht = min(img->height(),i+kernelHeight/2)-_max(0,i-kernelHeight/2)+1;
                for(int j=shiftSqKernel;j<resImg->width()-shiftSqKernel;++j){
                    // current pixel point is anchor
                    //if(((i-centerY)*(i-centerY)+(j-centerX)*(j-centerX)>=radiusSquared)){  //regionType skipped
                    if(abs(i-centerY)+abs(j-centerX)>radiusIntoRootTwo){
                        res++;          // not within ROI by Manhattan Distance
                    }
                    else{ 
                            int eff_wd = min(img->width(), j + kernelWidth/2)- _max(0,j-kernelWidth/2)+1;
                            currPtrImage = mat + rowSize*_max(0,i-kernelHeight/2)+_max(0,j-kernelWidth/2);
                            kerStartPt = kernel + _max(0,kernelHeight/2 -i)*kernelWidth + _max(0,kernelWidth/2-j);
                            float sum = 0;
                            for(int k=0; k<eff_ht;++k){
                                for(int l=0;l<eff_wd;++l){
                                   sum += *currPtrImage++ * *kerStartPt++*factor;
                                }
                                // shift the pointers
                                currPtrImage += rowSize - eff_wd-1;
                                kerStartPt += _max(0,j+kernelWidth/2-img->width());
                            }

                            
                            if(this->counter<this->flicker){
                                        this->limits[0] = this->limits[0]<sum?sum:this->limits[0];
                                        this->limits[1] = this->limits[1]>sum?sum:this->limits[1];
                                        this->counter++;
                                           
                                } 
                            *res++ = sum;                        
                    }
                }
                res += padOutput;
                
            }

                    
   
        }
        

};


#endif    

//----- end-of-file --- ( next line intentionally left blank ) ------------------

