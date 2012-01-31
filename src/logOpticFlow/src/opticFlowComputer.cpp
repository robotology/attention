// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Rea Francesco
  * email:francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
  */

/**
 * @file opticFlowComputer.cpp
 * @brief Implementation of the computation in a portion of the space (see opticFlowComputer.h)
 */

#include <iCub/opticFlowComputer.h>
#include <cstring>

//#define DEBUGDUMP

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

inline Matrix reshape(Matrix a,int row, int col) {
    int rows = a.rows();
    int cols = a.cols();
    Matrix b(row,col);
    if(rows * cols != col * row) {
        b.zero();
        return b;
    }
    int rb = 0;
    int cb = 0;
    for (int ra = 0; ra < rows; ra++) {
        for(int ca = 0; ca < cols; ca++) {
            b(rb,cb) = a(ra,ca);
            cb++;
            if(cb >= col){
                rb++;
                cb = 0;
            }
        }
    }
    return b;
}

opticFlowComputer::opticFlowComputer():Thread() { //RateThread(RATE_OF_INTEN_THREAD) {
    inputImage          = new ImageOf<PixelRgb>;
    filteredInputImage  = new ImageOf<PixelRgb>;
    extendedInputImage  = new ImageOf<PixelRgb>;        
   
    intensImg           = new ImageOf<PixelMono>;
    
    lambda  = 0.3f;
    resized = false;
    isYUV   = true;
    hasStartedFlag = false;
    

    double q = 0.5 * qdouble;
    for (int j = 0; j < 252; j ++) {
        double gammarad = (j / 180) * PI;
        fcos[j] = cos(gammarad / q);        
        fsin[j] = sin(gammarad / q);
    }
    width = 12; height = 12;
}

opticFlowComputer::opticFlowComputer(int i, int pXi,int pGamma, int n):Thread() {
 
    id = i;
    posXi = pXi;
    posGamma = pGamma;
    neigh = n ;
    width = 6; height = 6;
    dimComput = 6;

    fout = fopen("dump.txt","w+");

    fprintf(fout, "\n \n");

    double q = 0.5 * qdouble;
    for (int j = 0; j < 262; j ++) {
        double gamma = j;
        fcos[j] = cos(2 * PI - (gamma - 5) / q);
        fsin[j] = sin(2 * PI - (gamma - 5) / q);        
        //fprintf(fout, "%d %f %f \n",j, fcos[j], fsin[j]);
        
    }
    gradientHorConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(3,3,Sobel2DXgrad_small,0.5,.0,0);
    gradientVerConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(3,3,Sobel2DYgrad_small,0.1,.0,0);

    //resultU = new short;
    //resultV = new short;
    resultU = (short*) malloc(dimComput * dimComput * sizeof(short));
    resultV = (short*) malloc(dimComput * dimComput * sizeof(short));
    
}

opticFlowComputer::~opticFlowComputer() {
    printf("opticFlowComputer::~opticFlowComputer() \n");      
}

bool opticFlowComputer::threadInit() {

    q    = 0.5 * qdouble; // in rads
    a    = (1 + sin ( 1 / qdouble)) / (1 - sin(1 /qdouble));
    F    = a / (a - 1);
    rho0 = 1 / (pow (a, F) * (a - 1));

    printf(" \n correctly initialised variables \n");

    halfNeigh = floor(neigh / 2); 
    calcHalf  = (12 - 2 - 2 - 2) / 2;
    gammaStart = posGamma - 3; gammaEnd = posGamma + 3;
    xiStart    = posXi - 3;    xiEnd    = posXi + 3;
    dimComput = gammaEnd - gammaStart;   // 
    printf("dimComput %d %d %d \n", dimComput, posGamma, posXi);

    printf("correctly initialised indexes \n");

    Grxi    = new Matrix(5,5);     // matrix of xi gradient
    Grgamma = new Matrix(5,5);     // matrix of gamma gradient
    Grt     = new Matrix(5,5);     // matrix of temporal gradient
    H       = new Matrix(2,2);        
    s       = new Matrix(1,2);
    G       = new Matrix(2,1);
    B       = new Matrix(25,2);
    A       = new Matrix(25,2);    
    V       = new Matrix(2,2);
    S       = new Vector(2);
    K       = new Matrix(25,2);
    Km      = new Matrix(2,1);
    Kt      = new Matrix(2,25);
    of      = new Matrix(2,1);
    
    printf("correctly initialised the matrices \n");
    
    b       = new Matrix(25,1);
    c       = new Matrix(2,1);
    bwMat   = new Matrix(25,25);
    u       = new Matrix(dimComput,dimComput);
    v       = new Matrix(dimComput,dimComput);
    printf("trying to initialise the weight matrix \n");

    
    Vector wVec(25);
    wVec(0)  = 0.0103; wVec(1)  = 0.0463; wVec(2)  = 0.0764; wVec(3)  = 0.0463; wVec(4)  = 0.0103;
    wVec(5)  = 0.0463; wVec(6)  = 0.2076; wVec(7)  = 0.3422; wVec(8)  = 0.2076; wVec(9)  = 0.0463;
    wVec(10) = 0.0764; wVec(11) = 0.3422; wVec(12) = 0.2076; wVec(13) = 0.0463; wVec(14) = 0.0764;
    wVec(15) = 0.0463; wVec(16) = 0.2076; wVec(17) = 0.3422; wVec(18) = 0.2076; wVec(19) = 0.0463;
    wVec(20) = 0.0103; wVec(21) = 0.0463; wVec(22) = 0.0764; wVec(23) = 0.0463; wVec(24) = 0.0103;

    wMat    = new Matrix(25,25);
    wMat->diagonal(wVec);
    //wMat->eye();
    

    calculusIpl    = 0;
    calculusIpl32f = 0;
    represenIpl    = 0;
    temporalIpl    = 0;

    printf("correctly initialised the weight matrix \n");

    return true;
}

void opticFlowComputer::setCalculusImage(yarp::sig::ImageOf<yarp::sig::PixelMono> *img) {
    calculusPointer = img->getRawImage();
    calculusRowSize = img->getRowSize();
    int srcsizeWidth    = img->width();
    int srcsizeHeight   = img->height();
    calculusIpl     = cvCreateImage(cvSize(srcsizeWidth,srcsizeHeight),IPL_DEPTH_8U,1); 
    calculusIpl     = (IplImage*) img->getIplImage();
    calculusIpl32f  = cvCreateImage(cvSize(srcsizeWidth,srcsizeHeight),IPL_DEPTH_32F,1); 
    //convert im precision to 32f and process as normal:
    cvConvertScale(calculusIpl,calculusIpl32f,0.003922,0);  //  0.003922 = 1/255.0 
}

void opticFlowComputer::setTemporalImage(yarp::sig::ImageOf<yarp::sig::PixelMono> *img) {
    temporalPointer = img->getRawImage();
    //calculusRowSize = img->getRowSize();
    int srcsizeWidth    = img->width();
    int srcsizeHeight   = img->height();
    temporalIpl     = cvCreateImage(cvSize(srcsizeWidth,srcsizeHeight),IPL_DEPTH_8U,1); 
    temporalIpl     = (IplImage*) img->getIplImage();
    temporalIpl32f  = cvCreateImage(cvSize(srcsizeWidth,srcsizeHeight),IPL_DEPTH_32F,1); 
    //convert im precision to 32f and process as normal:
    cvConvertScale(temporalIpl,temporalIpl32f,0.003922,0);  //  0.003922 = 1/255.0 
}

void opticFlowComputer::setRepresenImage(ImageOf<PixelRgb>* img) {
    represenIpl = (IplImage*) img->getIplImage();
    represPointer = img->getRawImage();
    rowSize = img->getRowSize() / img->getPixelSize();
    represenImage = img;
}

void opticFlowComputer::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
}

std::string opticFlowComputer::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void opticFlowComputer::run() {   
    while(isRunning()) {               
        if(hasStartedFlag) {
             
            semCalculus->wait();
            //semTemporal->wait();
            estimateOF();       
            //semTemporal->post();
            semCalculus->post();

            
            //semRepresent->wait();
            //representOF();
            //semRepresent->post();
                        
            Time::delay(0.05);
        }        
    }
}

void opticFlowComputer::estimateOF(){
    // initialisation
    unsigned char *pNeigh, *nextRow, *nextPixel, *prevRow, *prevPixel;
    unsigned char *pCalc = calculusPointer;  
    unsigned char *pTemp = temporalPointer;
    unsigned char *pPrev;
    double k1, k2;
    short* tempResultU = resultU;
    short* tempResultV = resultV;

    //calculusRowSize = 264;
    //printf("rowSize %d \n", calculusRowSize);
  
    int i = 0;
    for (int gamma = 0; gamma < dimComput; gamma++) {
        for(int xi = 0; xi < dimComput; xi++) {
            i = 0;
            //printf("posXi %d posGamma %d xi %d  gamma %d \n", posXi, posGamma, xi, gamma);
            pCalc = calculusPointer + (posXi + xi - calcHalf) * calculusRowSize + (posGamma + gamma - calcHalf);
            pTemp = temporalPointer + (posXi + xi - calcHalf) * calculusRowSize + (posGamma + gamma - calcHalf);
            for (int dGamma = 0; dGamma < neigh; dGamma++) {
                for (int dXi = 0; dXi < neigh; dXi++) {
                    //printf("                   inner loop %d %d %d %d \n", dGamma, dXi, dGamma - halfNeigh, dXi - halfNeigh );
                    //printf("                   jump %d  because calculusRowSize %d  \n", (dXi - halfNeigh) * calculusRowSize + dGamma - halfNeigh, calculusRowSize);
                    pNeigh = pCalc + (dXi - halfNeigh) * calculusRowSize + dGamma - halfNeigh ;
                    pPrev  = pTemp + (dXi - halfNeigh) * calculusRowSize + dGamma - halfNeigh ;

                    //printf("log(a) = %f \n", log(a));
                    
                    
                    H->operator()(0,0) =      fcos[posGamma + gamma - calcHalf + dGamma - halfNeigh ]/log(a);
                    H->operator()(0,1) =      fsin[posGamma + gamma - calcHalf + dGamma - halfNeigh ]/log(a);
                    H->operator()(1,0) = -q * fsin[posGamma + gamma - calcHalf + dGamma - halfNeigh ];
                    H->operator()(1,1) =  q * fcos[posGamma + gamma - calcHalf + dGamma - halfNeigh ];
                    
                    /*                    
                    H->operator()(0,0) = 1;
                    H->operator()(0,1) = 0;
                    H->operator()(1,0) = 0;
                    H->operator()(1,1) = 1;
                    */

                    /*
                    if(posGamma + gamma - calcHalf + dGamma - halfNeigh == 130) {
                        printf("fcos = %f \n",fcos[posGamma + gamma - calcHalf + dGamma - halfNeigh]);
                        printf("H = \n %s \n %d ",H->toString().c_str(), calcHalf);
                        }
                    */
                    
                    nextRow = pNeigh + calculusRowSize;
                    prevRow = pNeigh - calculusRowSize;
                    
                    nextPixel = pNeigh + 1;
                    prevPixel = pNeigh - 1;

                    //Grxi->operator()(dXi,dGamma)    = ((*pNeigh - *nextRow)   + (*prevRow - *pNeigh)) / 2;
                    Grxi->operator()(dXi,dGamma)    = ( (float) *nextRow    - (float)*prevRow   ) ;
                    //Grgamma->operator()(dXi,dGamma) = ((*pNeigh - *nextPixel) + (*prevPixel - *pNeigh)) / 2;
                    Grgamma->operator()(dXi,dGamma) = ( (float) *nextPixel  - (float)*prevPixel ) ;
                    //unsigned char tdiff =(*pNeigh - *pPrev);
                    Grt->operator()(dXi,dGamma)     = (*pNeigh - *pPrev) ;

                        
                    
                    s->operator()(0,0) = Grxi->operator()(dXi,dGamma); 
                    s->operator()(0,1) = Grgamma->operator()(dXi,dGamma);
                    *G = *s * *H;
                    
                    /*
                    if((posXi - calcHalf + xi + dXi - halfNeigh == 100) &&
                        (posGamma + gamma - calcHalf + dGamma - halfNeigh == 130)){
                        //printf("rho %f power : %f  G =\n",rho0,rho0 * pow(a, posXi - calcHalf + xi + dXi - halfNeigh) );
                        printf("%d-%d= %d \n",*prevRow , *nextRow, *prevRow - *nextRow );
                        printf("%d-%d= %d \n",*prevPixel, *nextPixel, *prevPixel - *nextPixel );
                        printf("%f %f \n", Grxi->operator()(dXi,dGamma),Grgamma->operator()(dXi,dGamma) );
                        printf("%s \n", s->toString().c_str() );
                        printf("%s \n", G->toString().c_str() );
                    }
                    */
                    
                    
                    B->operator()(i,0) = (1 / (rho0 * pow(a, posXi - calcHalf + xi + dXi - halfNeigh))) * G->operator()(0,0);
                    B->operator()(i,1) = (1 / (rho0 * pow(a, posXi - calcHalf + xi + dXi - halfNeigh))) * G->operator()(0,1);                    
                                                      
                    /*
                      B->operator()(i,0) = G->operator()(0,0);
                    B->operator()(i,1) = G->operator()(0,1);
                    */
                    
                    i = i + 1;
                }
            }

                       
            *A = *wMat * *B;
            /*
            if((posGamma + gamma - calcHalf == 215)&&(posXi - calcHalf + xi == 112)) {
                 printf("%s \n", b->toString().c_str() );
            }
            */
            
            //printf("trying to reshape coefficients ater G =\n");
            //printf("%s \n",Grt->toString().c_str());
            *b = reshape(*Grt,neigh * neigh, 1);
            //printf("reshaped the matrix in to b =  \n");
            
            //printf(" %s \n", b->toString(1,1).c_str());            
            *b = -1 * *b;
            //if((posGamma + gamma - calcHalf == 215)&&(posXi - calcHalf + xi == 112)) {
            //     printf("                %s \n", b->toString().c_str() );
            //}
            //printf(" %s \n", b->toString(1,1).c_str());  
            *bwMat = *wMat * *b;                        
            //*bwMat = -1 * *b;

            SVD(*A,*K,*S,*V);            
            *Kt = K->transposed();
            *c  = *Kt * *bwMat;
            //printf("c = %s \n", c->toString().c_str());

            Km->operator()(0,0) = c->operator()(0,0) / S->operator()(0);
            Km->operator()(0,1) = c->operator()(1,0) / S->operator()(1);
            

            *of = *V * *Km;
            //printf("of =%s \n", of->toString().c_str());
            u->operator()(xi,gamma) = of->operator()(0,0);
            v->operator()(xi,gamma) = of->operator()(0,1);

            /*
            double ofu = V->operator()(0,0) * k1;
            u->operator()(xi,gamma) = ofu ;
            //v->operator()(xi,gamma) 
            double ofv  = V->operator()(1,0) * k2;     
            v->operator()(xi,gamma) = ofv ;
            */

            *tempResultU = (short) floor(u->operator()(xi,gamma));
            tempResultU++;
            *tempResultV = (short) floor(v->operator()(xi,gamma));
            tempResultV++;
            
            
#ifdef DEBUGDUMP
            fprintf(fout,"%05f %05f \n", u->operator()(xi,gamma), v->operator()(xi, gamma));
#endif
            //printf(" optic flow = (%d, %d) \n", *tempResultU, *tempResultV);
        }
    }
#ifdef DEBUGDUMP
    fprintf(fout,"-1 -1 \n");
#endif
    
    //printf("setting the portion for %d computer \n", id);
    pt->setPortionU(id,resultU);
    pt->setPortionV(id,resultV);
    //printf("success in setting the portion \n");
    
}

void opticFlowComputer::representOF(){
    
    unsigned char* tempPointer;
    
    tempPointer = represPointer;
    //int rowSizeC = (((252 + 5)/ 8)+ 1)  * 8;
    //printf("representing OF %d %d  \n", rowSizeC, rowSize);

    if(represPointer!=0) {
        //representing the limits
        //printf("image pointer %x %d %d %d \n", represPointer, posXi, posGamma, width);
        /*
        tempPointer  = represPointer + (posXi * rowSize + posGamma) * 3;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 0;   tempPointer++;
        */
        
        /*
        tempPointer  = represPointer + ((posXi - width) * rowSize + (posGamma - height)) * 3;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        
        tempPointer  = represPointer + ((posXi - width) * rowSize + (posGamma + height)) * 3;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        
        tempPointer  = represPointer + ((posXi + width ) * rowSize + (posGamma - height)) * 3;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        
        tempPointer  = represPointer + ((posXi + width ) * rowSize + (posGamma + height)) * 3;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        */

        //representing the vectors        
        //CvScalar colorScalar = CV_RGB(50,0250)
        
        double sumGamma, sumXi;
        int valueGamma, valueXi;
        int meanGamma,  meanXi;
        int countGamma = 0, countXi = 0;
        sumGamma = 0; sumXi = 0;
        double uSingle, vSingle;
        tempPointer = represPointer + ((posXi - calcHalf) * rowSize + (posGamma - calcHalf)) * 3 ;
        for(int i = 0; i < dimComput; i++) {
            for(int j =0; j< dimComput; j++) {
                uSingle = u->operator()(i,j) * 1.0;
                vSingle = v->operator()(i,j) * 1.0;
                valueGamma = (int) floor(uSingle / 1.0);
                valueXi    = (int) floor(vSingle / 1.0);
                
                //tempPointer  = represPointer + (((posXi + j - calcHalf )* rowSize) + posGamma + i - calcHalf) * 3;
                if((abs(uSingle) > 40)||(abs(vSingle) > 40)) {
                    //cvLine(represenIpl,
                    //               cvPoint(posGamma + i - calcHalf, posXi + j - calcHalf), 
                    //               cvPoint(posGamma + i - calcHalf + valueGamma, posXi + j - calcHalf + valueXi),
                    //               cvScalar(0,0,255,0));  

                    *tempPointer = 0;   tempPointer++;
                    *tempPointer = 255; tempPointer++;
                    *tempPointer = 0;   tempPointer++;
                }
                else { 
                        *tempPointer = 0;   tempPointer++;
                        if(*tempPointer > 0 ){
                            *tempPointer = *tempPointer - 1; 
                        }
                        else{
                            *tempPointer = *tempPointer;
                        }
                        tempPointer++;
                        *tempPointer = 0;   tempPointer++;

                        //tempPointer += 3;                    
                }
                
                //printf("Single: %f %f \n", uSingle, vSingle);
                /*
                if((v== v)&&(v!=0)) {
                    sumGamma += uSingle;
                    countGamma++;
                }   
                if((u==u)&&(u!=0)){
                    sumXi    += vSingle;
                    countXi++;
                }
                */
                
                
                //
                //

                /*
                if(
                   (posGamma + valueGamma < 0)       || (posXi + valueXi < 0) ||
                   (posGamma + valueGamma > rowSize) || (posXi + valueXi > 152)
                   ){
                        //printf("line out of image boundaries \n");
                }
                else {
                    if((valueGamma != 0)&&(valueXi != 0)) {
                       
                            //printf("endPoint %f %f %d %d \n",uSingle, vSingle, endPointX, endPointY);
                            //cvLine(represenIpl,
                            //       cvPoint(posGamma + i - calcHalf, posXi + j - calcHalf), 
                            //       cvPoint(posGamma + i - calcHalf + valueGamma, posXi + j - calcHalf + valueXi),
                            //       cvScalar(0,0,255,0));   
                            
                            //tempPointer  = represPointer + (posXi  * rowSize + posGamma)  * 3;
                            
                            //tempPointer  = represPointer + (((posXi + j - calcHalf )* rowSize) + posGamma + i - calcHalf) * 3;
                            //*tempPointer = 255; tempPointer++;
                            //*tempPointer = 0  ; tempPointer++;
                            //*tempPointer = 0  ; tempPointer++;
                       
                    }
                }
                */


            } //end for j
            tempPointer += (rowSize - dimComput) * 3;
        } // end for i


        //printf("sumGamma %f sumXi %f  countGamma %d  countXi %d \n", sumGamma, sumXi, countGamma, countXi);
        meanGamma  =  (int) (sumGamma / countGamma);
        meanXi     =  (int) (sumXi    / countXi   );
        //printf("value: %d %d \n", valueGamma, valueXi);
        //printf(" mean ( %d %d) \n", meanGamma, meanXi);
        //if((meanGamma > 5) || (meanXi > 5)) {
        //    cvLine(represenIpl,
        //           cvPoint(posGamma , posXi), 
        //           cvPoint(posGamma + meanGamma, posXi + meanXi),
        //           cvScalar(255,0,0,0));
        //}        
    }
    else {
        printf("null pointer \n");
    }
}

void opticFlowComputer::resize(int width_orig,int height_orig) {
 
    //resizing yarp image 
    filteredInputImage->resize(width, height);
    extendedInputImage->resize(width, height);
    
   
    //edges->resize(width, height);
    intensImg->resize(width, height);
    //    unXtnIntensImg->resize(this->width_orig,this->height_orig);    

    
    
    // allocating for CS ncsscale = 4;   

    //cs_tot_32f  = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_32F, 1  );
    //colcs_out   = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    //ycs_out     = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    //scs_out     = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    //vcs_out     = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    
    isYUV = true; 
}

void opticFlowComputer::filterInputImage() {
    
    int i;
    const int szInImg = inputImage->getRawImageSize();
    unsigned char * pFilteredInpImg = filteredInputImage->getRawImage();
    unsigned char * pCurr = inputImage->getRawImage();
    int pad = inputImage->getPadding();
    float lambda = .5f;
    const float ul = 1.0f - lambda;
    for (i = 0; i < szInImg; i++) { // assuming same size
        *pFilteredInpImg = (unsigned char)(lambda * *pCurr++ + ul * *pFilteredInpImg++ + .5f);
    }
}


void opticFlowComputer::extender(int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*extendedInputImage, *filteredInputImage, maxSize);    
}

void opticFlowComputer::extractPlanes() {
    /*
    //chromeThread->setFlagForDataReady(false);           
    // We extract color planes from the RGB image. Planes are red,blue, green and yellow (AM of red & green)
    uchar* shift[4];
    uchar* YUV[3];
    uchar* unXtnYUV[3];
    int padInput;
    int padUnX;
    int padUnXtnYUV;
    int padMono;
    uchar* tmpIntensityImage;
    uchar* ptrIntensityImg;
    uchar* ptrUnXtnIntensImg;
    uchar* inputPointer;    
    
    // Pointers to raw plane image
    shift[0] = (uchar*) redPlane->getRawImage(); 
    shift[1] = (uchar*) greenPlane->getRawImage(); 
    shift[2] = (uchar*) bluePlane->getRawImage(); 
    shift[3] = (uchar*) yellowPlane->getRawImage();

    YUV[0] = (uchar*) Yplane->getRawImage(); 
    YUV[1] = (uchar*) Uplane->getRawImage(); 
    YUV[2] = (uchar*) Vplane->getRawImage();

    unXtnYUV[0] = (uchar*) unXtnYplane->getRawImage(); 
    unXtnYUV[1] = (uchar*) unXtnUplane->getRawImage(); 
    unXtnYUV[2] = (uchar*) unXtnVplane->getRawImage(); 

 
    ptrIntensityImg   = (uchar*) intensImg->getRawImage();
    ptrUnXtnIntensImg = (uchar*) unXtnIntensImg->getRawImage();
    inputPointer      = (uchar*) extendedInputImage->getRawImage();
    padInput          = extendedInputImage->getPadding();
    padMono           = redPlane->getPadding();
    padUnX            = unXtnIntensImg->getPadding();
    padUnXtnYUV       = unXtnYplane->getPadding();
    

    const int h = extendedInputImage->height();
    const int w = extendedInputImage->width();

    
    for(int r = 0; r < h; r++) {               
        for(int c = 0; c < w; c++) {
            // assuming order of color channels is R,G,B. For some format it could be B,G,R.
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *shift[3]++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            *ptrIntensityImg = ONE_BY_ROOT_THREE * sqrt(*shift[0] * *shift[0] +*shift[1] * *shift[1] +*shift[2] * *shift[2]);
            

            // RGB to Y'UV conversion
            float red = (float)*shift[0];
            float green = (float)*shift[1];
            float blue = (float)*shift[2];

            int Y, U, V;
            Y = 0.299*red + 0.587*green + 0.114*blue;
            U = (blue-*YUV[0])*0.564 +128.0;
            V = (red-*YUV[0])*0.713 +128.0;
            
            *YUV[0] = max(16,min(235,Y));
            *YUV[1] = max(16,min(235,U));
            *YUV[2] = max(16,min(235,V));
            

            if(r>=maxKernelSize && c >=maxKernelSize && c< w-maxKernelSize){
                *ptrUnXtnIntensImg++ = *ptrIntensityImg;
                *unXtnYUV[0]++ = *YUV[0];
                *unXtnYUV[1]++ = *YUV[1];
                *unXtnYUV[2]++ = *YUV[2];                
            }

            ptrIntensityImg++;
            YUV[0]++;
            YUV[1]++;
            YUV[2]++;
            shift[0]++;
            shift[1]++;
            shift[2]++;
        }
        // paddings
        inputPointer += padInput;
        ptrIntensityImg += padMono;
        if(r>=maxKernelSize){
            
            ptrUnXtnIntensImg += padUnX;
            unXtnYUV[0] += padUnXtnYUV;
            unXtnYUV[1] += padUnXtnYUV;
            unXtnYUV[2] += padUnXtnYUV;
        }
        shift[0] += padMono;
        shift[1] += padMono;
        shift[2] += padMono;
        shift[3] += padMono;
        YUV[0] += padMono;
        YUV[1] += padMono;
        YUV[2] += padMono;       
                
    } 
    */

}

void opticFlowComputer::filtering() {

    /*
    // We gaussian blur the image planes extracted before, one with positive Gaussian and then negative
    
    //Positive
    // This is calculated via first scale of YUV planes*/

    /*tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Rplus);
    

    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Bplus);

    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(greenPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Gplus);*/   
    
}



void opticFlowComputer::colorOpponency(){
    /*
    // get opponent colors for eg R+G- from R+ and G- channels
    // Finding color opponency now

    ImageOf<PixelMono>& coRG = colorOpp1Port.prepare();
    ImageOf<PixelMono>& coGR = colorOpp2Port.prepare();
    ImageOf<PixelMono>& coBY = colorOpp3Port.prepare();

    coRG.resize(width,height);
    coGR.resize(width,height);
    coBY.resize(width,height);
    
    
    uchar* pRG = coRG.getRawImage();
    uchar* pGR = coGR.getRawImage();
    uchar* pBY = coBY.getRawImage();

    uchar* rPlus = Rplus->getRawImage();
    uchar* rMinus = Rminus->getRawImage();
    uchar* gPlus = Gplus->getRawImage();
    uchar* gMinus = Gminus->getRawImage();
    uchar* bPlus = Bplus->getRawImage();
    uchar* yMinus = Yminus->getRawImage();

    int padChannel = Rplus->getPadding();
    int padOpponents = coRG.getPadding();

    for(int r = 0; r < height; r++) {
        for(int c = 0; c < width; c++) {
            
            *pRG++ = ((*rPlus >> 1) + 128 - (*gMinus >> 1) );
            *pGR++ = ((*gPlus >> 1) + 128 - (*rMinus >> 1) );
            *pBY++ = ((*bPlus >> 1) + 128 - (*yMinus >> 1) );

            rMinus++;
            rPlus++;
            gMinus++;
            gPlus++;
            yMinus++;
            bPlus++;
        }

        rMinus += padChannel;
        rPlus  += padChannel;
        gMinus += padChannel;
        gPlus  += padChannel;
        yMinus += padChannel;
        bPlus  += padChannel;
        pRG += padOpponents;
        pGR += padOpponents;
        pBY += padOpponents;

    }

    if(colorOpp1Port.getOutputCount()) {
        colorOpp1Port.write();
    }
    if(colorOpp2Port.getOutputCount()) {
        colorOpp2Port.write();
    }
    if(colorOpp3Port.getOutputCount()) {
        colorOpp3Port.write();
    }
    
#ifdef DEBUG_OPENCV
    cvNamedWindow("ColorOppRG");
    cvShowImage("ColorOppRG", (IplImage*)coRG.getIplImage());
    cvNamedWindow("ColorOppGR");
    cvShowImage("ColorOppGR", (IplImage*)coGR.getIplImage());
    cvNamedWindow("ColorOppBY");
    cvShowImage("ColorOppBY", (IplImage*)coBY.getIplImage());
    cvWaitKey(2);
#endif
    */

}

void opticFlowComputer::centerSurrounding(){
        
    // YofYUVpy->zero();
    // VofYUVpy->zero();
    // UofYUVpy->zero();
    // Rplus->zero();
    // Bplus->zero();
    // Gplus->zero();        
        
        //float YUV2RGBCoeff[9]={1, 0, 1.403,
                                //1, -.344, -.714,
                                //1, 1.77, 0
                                //};
                                //{-2.488,  3.489,  0.000,
                                 //3.596, -1.983, -0.498,
                                //-3.219,  1.061,  2.563};
                                       
       
}

void opticFlowComputer::addFloatImage(IplImage* sourceImage, CvMat* cvMatAdded, double multFactor, double shiftFactor){

    IplImage stub, *toBeAddedImage;
    toBeAddedImage = cvGetImage(cvMatAdded, &stub);
    assert( sourceImage->width == toBeAddedImage->width && sourceImage->height == toBeAddedImage->height );
    float *ptrSrc, *ptrToBeAdded;
    ptrSrc = (float*)sourceImage->imageData;
    ptrToBeAdded = (float*)toBeAddedImage->imageData;
    int padImage = sourceImage->widthStep/sizeof(float) - sourceImage->width; //assuming monochromatic uchar image
    for(int i=0 ; i< sourceImage->height; ++i){
        for(int j=0; j< sourceImage->width; ++j){
            *ptrSrc = *ptrSrc + (multFactor* *ptrToBeAdded + shiftFactor); // in-place
            ptrSrc++;
            ptrToBeAdded++;
        }
        ptrSrc += padImage;
        ptrToBeAdded += padImage;
   }   
}

void opticFlowComputer::onStop(){
    resized = false;

    printf("opticFlowComputer::onStop()");

    // deallocating resources
    delete inputImage;
    delete filteredInputImage;
    delete extendedInputImage;
    delete intensImg;
    
    printf("correctly freed memory for images \n");
    
    delete Grxi;
    delete Grgamma;     // matrix of gamma gradient
    delete Grt; 
    delete H;           
    delete s;   
    delete G;
    delete B;   
    delete A;
    delete K;
    delete s;
    delete V;
    delete Km;
    delete of;
        
    delete b;      
    delete bwMat;  
    delete u;      
    delete v;      
    
    printf("correctly deleting matrices \n");


    free(resultU);
    free(resultV);

    /*imagePortIn.interrupt();
    imagePortOut.interrupt();
    intenPort.interrupt();
    imagePortIn.close();
    imagePortOut.close();
    intenPort.close();

    colorOpp1Port.interrupt();
    colorOpp2Port.interrupt();
    colorOpp3Port.interrupt();
    
    colorOpp1Port.close();
    colorOpp2Port.close();
    colorOpp3Port.close();
    */
 
    printf("Done with releasing earlyVision thread.\n");
}


void opticFlowComputer::threadRelease() {    
    

    resized = false;

    // deallocating resources
    delete inputImage;
    delete filteredInputImage;
    delete extendedInputImage;       
    delete intensImg;

    if(temporalIpl != 0) {
        //cvRelease(temporalIpl);
        delete temporalIpl;
    }
    if(calculusIpl!=0) {
        //cvRelease(calculusIpl);
        delete calculusIpl;
    }
    if(calculusIpl32f!=0) {
        //cvRelease(calculusIpl32f);
        delete calculusIpl32f;
    }
    if(represenIpl!=0) {
        //cvRelease(represenIpl);
        delete represenIpl;
    }    
    
    printf("correctly deleting the images \n");
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    intenPort.interrupt();
    imagePortIn.close();
    imagePortOut.close();
    intenPort.close();

    colorOpp1Port.interrupt();
    colorOpp2Port.interrupt();
    colorOpp3Port.interrupt();
    
    colorOpp1Port.close();
    colorOpp2Port.close();
    colorOpp3Port.close();
 
    printf("Done with releasing earlyVision thread.\n");
    
}




