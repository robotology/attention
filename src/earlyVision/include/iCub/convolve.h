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
        int kernelHeight;       // height of the kernel. When 1D kernel is applied in horizontal, this is length       
        float* kernel;          // pointer to kernel values
        int direction;          // direction in which kernel is applied,(0,1,2) for (horizontal,vertical,both)
        float factor;           // scaling factor applied to kernel multiplication
        int shift;              // shift in values applied to kernel multiplication
        bool kernelIsDefined;   // flag to check that kernel values are set properly

        // Not allowing copying and assignment
        void operator=(const convolve&);
        convolve(const convolve&);

    public:
        convolve(){
            this->kernelWidth   = 0;
            this->kernelHeight  = 0;
            this->kernel        = NULL;
            this->direction     = -1;
            this->factor        = 0;
            this->shift         = 0;
            this->kernelIsDefined = false; 
        };

    /**
     * For a two dimensional kernel, for inseparable kernels 
     * @param width width of kernel
     * @param height height of kernel
     * @param kernel pointer to float array representing kernel values
     * @param scale scaling factor applied to kernel multiplication
     * @param shift shift in values applied to kernel multiplication
     */
        convolve(int width,int height,float* kernel, float scale,int shift){
        
            this->kernelWidth   = width;
            this->kernelHeight  = height;
            this->kernel        = kernel;
            this->direction     = 2;
            this->factor        = scale;
            this->shift         = shift;
            this->kernelIsDefined = true;               //LATER: more assertions
        };
    /**
     * For a linear kernel. For separable 2D kernels, this can be efficient way to do
     * @param length legnth of vector representing values of kernel
     * @param kernel pointer to float array representing kernel values
     * @param direction direction of the kernel, 0 implies horizontal and 1 implies vertical
     * @param scale scaling factor applied to kernel multiplication
     * @param shift shift in values applied to kernel multiplication
     */
        convolve(int length,float* kernel,int direction,float scale,int shift){
        
            if(direction == 0) {
                this->kernelHeight = 0;
                this->kernelWidth   = length;                
            }
            else if(direction == 1){
                this->kernelHeight = length;
                this->kernelWidth   = 0;                
            }
            this->kernel        = kernel;
            this->direction     = direction;
            this->factor        = scale;
            this->shift         = shift;
            this->kernelIsDefined = true;               //LATER: more assertions
        };
        ~convolve(){
            //nothing
        };

        void setKernelParameters(int width,int height,float* kernel,float scale,int shift){
        
            this->kernelWidth   = width;
            this->kernelHeight  = height;
            this->kernel        = kernel;
            this->direction     = direction;
            this->factor        = scale;
            this->shift         = shift;
            this->kernelIsDefined = true;               //LATER: more assertions
        };


        // Convolution of a kernel with image, in given direction
        void convolve1D(inputImage* img,outputImage* resImg){
            assert(kernelIsDefined);
            float maxPixelVal = 0;
            float minPixelVal = -256;
            int rowSize         = img->getRowSize()/sizeof(ptrInput);
            int resRowSize = resImg->getRowSize()/sizeof(ptrOutput);
            ptrInput* mat = (ptrInput*)img->getRawImage();
            ptrOutput* res = (ptrOutput*)resImg->getRawImage();
            int rowPos = 0; int pixPos =0;    
            
            if(this->direction == 0){ //horizontal convolution
                float* midVec = kernel + this->kernelWidth/2; // middle of linear kernel    
                for(int i=0;i<resImg->height();++i){
                    for(int j=0;j<resImg->width();++j){
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
                        if(sum>maxPixelVal) maxPixelVal=sum;
                        else if(sum<minPixelVal) minPixelVal=sum;               
                        *(res+i*resRowSize+j)=sum;//+ shift;//<0?0: sum>maxVal? maxVal:sum;
                    }
                } 
            } 
            else if(this->direction == 1){
                float* midVec = kernel + this->kernelHeight/2; // middle of linear kernel    
                for(int i=0;i<resImg->height();++i){
                    for(int j=0;j<resImg->width();++j){
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
                        *(res+i*resRowSize+j)=sum;
                    }
                } 
            }
           else; // wrong direction
            
        }

        void convolve2D(inputImage* img,outputImage* resImg){
            
            assert(kernelIsDefined);
            float maxPixelVal = 0;
            float minPixelVal = -256;
            int rowSize         = img->getRowSize()/sizeof(ptrInput);
            int resRowSize = resImg->getRowSize()/sizeof(ptrOutput);
            ptrInput* mat = (ptrInput*)img->getRawImage();
            ptrOutput* res = (ptrOutput*)resImg->getRawImage();
            int rowPos = 0; int pixPos =0;
            ptrInput* currPtrImage = (ptrInput*)img->getRawImage();
            int padOutput = resImg->getPadding();
            float* kerStartPt = this->kernel;
            for(int i=0;i<resImg->height();++i){
                int eff_ht = min(img->height(),i+kernelHeight/2)-max(0,i-kernelHeight/2)+1;
                for(int j=0;j<resImg->width();++j){
                    // current pixel point is anchor
                    int eff_wd = min(img->width(), j + kernelWidth/2)- max(0,j-kernelWidth/2)+1;
                    currPtrImage = mat + resRowSize*max(0,i-kernelHeight/2)+max(0,j-kernelWidth/2);
                    kerStartPt = kernel + max(0,kernelHeight/2 -i)*kernelWidth + max(0,kernelWidth/2-j);
                    //printf("i%dj%deff ht%d,wd%d \n",i,j,eff_ht,eff_wd);
                    float sum = 0;
                    for(int k=0; k<eff_ht;++k){
                        for(int l=0;l<eff_wd;++l){
                           sum += *currPtrImage++ * *kerStartPt++;
                        }
                        // shift the pointers
                        currPtrImage += resRowSize - eff_wd-1;
                        kerStartPt += max(0,j+kernelWidth/2-img->width());
                    }

                    *res++ = sum*factor;
                }
                res += padOutput;
            }

                    
   
        }
        

};
  
#endif    

//----- end-of-file --- ( next line intentionally left blank ) ------------------

