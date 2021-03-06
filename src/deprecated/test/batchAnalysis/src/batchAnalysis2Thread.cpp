// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*                                                                                                                                  
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file batchAnalysis2Thread.cpp
 * @brief Implementation of the eventDriven thread (see batchAnalysis2Thread.h).
 */

#include <iCub/batchAnalysis2Thread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;

batchAnalysis2Thread::batchAnalysis2Thread() {
    robot = "icub";        
}

batchAnalysis2Thread::batchAnalysis2Thread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

batchAnalysis2Thread::~batchAnalysis2Thread() {
    // do nothing
}

bool batchAnalysis2Thread::threadInit() {

    //opening port
    outputPortTime.open      (getName("/time:o").c_str());      //ddd

    // initialization of the attributes
    width  = WIDTH;
    height = HEIGHT;
    idle   = false;
    throwAway = false;
    firstProcessing=true;
    numberProcessing=0;
    setAlgorithm(ALGO_FB);
    timeCounter=0;

    //inputImage = new ImageOf<PixelRgb>();
    //inputImage->resize(width, height);

    processingImage = new ImageOf<PixelRgb>();
    processingImage->resize(width, height);

    // opening the port for direct input
    //if (!inputPort.open(getName("/image:i").c_str())) {
    //    yError("unable to open port to receive input");
    //    return false;  // unable to open; let RFModule know so that it won't run
    //}  

    // opening the port for direct output
    if (!outputPort.open(getName("/result:o").c_str())) {
        yError("unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }   

    //starting the plotterThread      bbb
    pt = new plotterThread();
    pt->setName(getName("").c_str());
    pt->start();

    //starting the featExtractorThread        aaa
    fet = new featExtractorThread();
    fet->setName(getName("").c_str());
    fet->start();

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void batchAnalysis2Thread::setName(string str) {
    this->name=str;
}


std::string batchAnalysis2Thread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void batchAnalysis2Thread::setInputPortName(string InpPort) {
    
}

void batchAnalysis2Thread::visualizationResume(){
    pt->resume();
}

void batchAnalysis2Thread::visualizationSuspend(){
    pt->suspend();
}

bool batchAnalysis2Thread::test(){
    bool reply = true;

    if(pt==NULL){
        reply = false;
    }
    else {
        reply &= pt->test();
    }

    reply  &= Network::exists(getName("/image:i").c_str());

    return reply;
}

void batchAnalysis2Thread::run() {
    while (isStopping() != true) {
        bool result;

    if(outputPortTime.getOutputCount()) {
        Bottle& outbotTime = outputPortTime.prepare();
        outbotTime.clear();
        outbotTime = contentBottleTime;
        outputPortTime.write();
    }


        if(!idle){
            alg="Farneback";
            //sequenceID=1;
            for (sequenceID=53; sequenceID<=54; sequenceID++){      //<=8
                //simplest case
                //k_min=0;
                //k_max=500;
                //if (sequenceID==1){seq="reaching/leftDumper_a/";}
                //if (sequenceID==3){seq="reaching/leftDumper_b/";}
                //if (sequenceID==2){seq="roll/leftDumper_a/";}
                //if (sequenceID==4){seq="roll/leftDumper_b/";}

                //train

                //test
                if (sequenceID==21){
                    seq="bio/rollpasta/rollpasta_Alessia_lowvel_2/leftDumper/";
                    k_min=0;
                    k_max=299; //in tuttoo manda la bottiglia avanti e indietro per 9 volte
                }
                if (sequenceID==22){
                    seq="bio/rollpasta/rollpasta_Oskar_lowvel_2/leftDumper/";
                    k_min=13;
                    k_max=295; //in tuttoo manda la bottiglia avanti e indietro per 9 volte, leggermente piu veloce di alessia e alessandra, quindi un po'meno frame di 300. Inoltre si vede meno la testa
                }
                if (sequenceID==23){
                    seq="bio/pointing/pointing_Alessia_2/leftDumper/";
                    k_min=63;
                    k_max=353; //7 movimenti di pointing
                }
                if (sequenceID==24){
                    seq="bio/pointing/pointing_Oskar_2/leftDumper/";
                    k_min=62;
                    k_max=276; //7 movimenti di pointing
                }
                if (sequenceID==25){
                    seq="bio/mixing/mixing_Alessia_2/leftDumper/";
                    k_min=1;
                    k_max=222; //25 giri (nel training ho usato quasi 29 giri di Alessandra, purtroppo probabilmente io ero andata un pochino piu lenta e quindi avevamo ripreso meno giri. Inoltre, in questo video il cucchiaino si vede poco, ma cmq si muove la mano
                }
                if (sequenceID==26){
                    seq="bio/mixing/mixing_Oskar_2/leftDumper/";
                    k_min=2;
                    k_max=200; //quasi 29 giri(come nel training con Alessandra). Anche qui, come nel traning di Alessandra,  si vede bene il cucchiaio
                }
                if (sequenceID==27){
                    seq="bio/transporting/transporting_Alessia_slow_2/leftDumper/";
                    k_min=50;
                    k_max=262;  //6 spostamenti di oggetto
                }
                if (sequenceID==28){
                    seq="bio/transporting/transporting_Oskar_slow_2/leftDumper/";
                    k_min=26;
                    k_max=301;  //6 spostamenti di oggetto
                }
                if (sequenceID==29){
                    seq="bio/writing/writing_Alessia_2/leftDumper/";
                    k_min=18;
                    k_max=234;  //tre e mezzo scritte
                }
                if (sequenceID==30){
                    seq="bio/writing/writing_Oskar_2/leftDumper/";
                    k_min=22;
                    k_max=263;  //tre e mezzo scritte
                }

                //test  on change of velocity
                if (sequenceID==31){
                    seq="bio/rollpasta/rollpasta_Alessandra_fastvel_2/leftDumper/";
                    k_min=3;
                    k_max=173;  //in tutto manda la bottiglia avanti e indietro per 9 volte
                }
                if (sequenceID==32){
                    seq="bio/transporting/transporting_Alessandra_fast_2/leftDumper/";
                    k_min=0;
                    k_max=191;  //6 spostamenti di oggetto
                }

                //test on change of velocity AND subjects
                if (sequenceID==33){
                    seq="bio/rollpasta/rollpasta_Alessia_fastvel_2/leftDumper/";
                    k_min=36;
                    k_max=215;  //in tutto manda la bottiglia avanti e indietro per 9 volte
                }
                if (sequenceID==34){
                    seq="bio/rollpasta/rollpasta_Oskar_fastvel_2/leftDumper/";
                    k_min=8;
                    k_max=180;  //in tutto manda la bottiglia avanti e indietro per 9 volte
                }
                if (sequenceID==35){
                    seq="bio/transporting/transporting_Alessia_fast_2/leftDumper/";
                    k_min=23;
                    k_max=160;  //6 spostamenti di oggetto
                }
                if (sequenceID==36){
                    seq="bio/transporting/transporting_Oskar_fast_2/leftDumper/";
                    k_min=2;
                    k_max=176;  //6 spostamenti di oggetto
                }
                //test on nonbio
                //test on change of velocity (ma c'� cmq accelerazione iniziale come in trainingset)
                if (sequenceID==37){
                    seq="notBio/roll/zigzag_slower/zigzag_const_2/leftDumper/"; 
                    k_min=58;
                    k_max=358;  //300 frames, si vede accelerazione
                }
                //test on change of pattern (same vel)
                if (sequenceID==38){
                    seq="notBio/roll/sin_faster/sin_notconst_2/leftDumper/";
                    k_min=60;
                    k_max=360;  //300 frames, si vede accelerazione
                }
                //test on change of velocity profile (train which follows 2/3power law)
                if (sequenceID==39){
                    seq="notBio/ellipse_train/train_following_law_2/leftDumper/";
                    k_min=12;
                    k_max=394;  //fare 3 giri partendo  da pezzo retto davanti in mezzo
                }
                //test on change of velocity (slower than training)
                if (sequenceID==40){
                    seq="notBio/ellipse_train/train_superslow_2/leftDumper/";
                    k_min=155;
                    k_max=798;  //fare 3 giri partendo  da pezzo retto davanti in mezzo
                }
                //test on change of trajectory (circle instead of ellipse)
                if (sequenceID==41){
                    seq="notBio/circle_train/train_fast_2/leftDumper/"; 
                    k_min=11;
                    k_max=141;  //fare 3 giri partendo  da pezzo davanti in mezzo
                }
                if (sequenceID==42){
                    seq="notBio/circle_train/train_slow_2/leftDumper/"; 
                    k_min=32;
                    k_max=271;  //fare 3 giri partendo  da pezzo davanti in mezzo
                }
                if (sequenceID==43){
                    seq="notBio/circle_train/train_superslow_2/leftDumper/"; 
                    k_min=57;
                    k_max=470;  //fare 3 giri partendo  da pezzo davanti in mezzo
                }

                //test on old Nico's sequences
                if (sequenceID==44){
                    seq="GestAle_Front_Left/";
                    k_min=40;
                    k_max=340;    //300 frames
                }
                if (sequenceID==45){
                    seq="GestGab_Front_Left/";
                    k_min=30;
                    k_max=330;    //300 frames
                }
                if (sequenceID==46){
                    seq="GestNico_Front_Left/";
                    k_min=28;
                    k_max=328;    //300 frames
                }
                if (sequenceID==47){
                    seq="LiftAle100_Front_Left/";
                    k_min=78;
                    k_max=487;  //6spostam di ogg
                }
                if (sequenceID==48){
                    seq="LiftGab100_Front_Left/";
                    k_min=15;
                    k_max=450;   //6spostam di ogg
                }
                if (sequenceID==49){
                    seq="LiftNico100_Front_Left/";
                    k_min=15;
                    k_max=581;    //6spostam di ogg
                }
                if (sequenceID==50){
                    seq="Macchinina_Front_Left/";
                    k_min=1;
                    k_max=431;   //6movim di macchinina
                }
                if (sequenceID==51){
                    seq="Pendolo_Front_Left/";
                    k_min=1;
                    k_max=297;      //12 volte a destra e sx (1 a dx,1 a sx, 1 a dx...)
                }


                if (sequenceID==52){
                    seq="LiftAle400_Front_Left/";
                    k_min=20;
                    k_max=552;  //6spostam di ogg
                }
                if (sequenceID==53){
                    seq="LiftGab400_Front_Left/";
                    k_min=20;
                    k_max=487;   //6spostam di ogg
                }
                if (sequenceID==54){
                    seq="LiftNico400_Front_Left/";
                    k_min=1;
                    k_max=569;    //6spostam di ogg
                }










                for ( k =k_min; k<k_max; k=k+1){

                    acquisitionSequence();   //it takes the one image            //inputImage = inputPort.read(true);

                    //if (throwAway){
                    //    throwAway = false;
                    //}
                    //else {
                        double timeStart = Time::now();
                        //processing
                        result = processing();               //generates the outputImage which is what we want to plot
                        double timeStopProcessing = Time::now();
                        timeDiffProcessing =timeStopProcessing-timeStart;
                        //yDebug("timeDiff %f", timeDiffProcessing);


                        //bbb
                        pt->copyImage(processingImage);
                        pt->copyU(U);                       //I have instantiated an  object p of type plotterThread, and now I can call the function of this class (copyU)
                        pt->copyV(V);
                        pt->copyM(Maskt);

                        fet->copyAll(U,V,Maskt,sequenceID,k);    //aaa
                        fet->setFlag();             //aaa

                        /*
                        if (outputPort.getOutputCount()) {
                            //yDebug("debug");
                            ImageOf<PixelMono> &c = outputPort.prepare();
                            c.resize(320,240);
                            c.copy(*outputImage);
                            outputPort.write();
                            //yDebug("debug2");
                        }
                        */
                        double timeStop = Time::now();
                        double timeDiff =timeStop-timeStart;
                        //yDebug("timeDiff %f", timeDiff);

                        throwAway = true;
                    //}
                }
            }
        }
    }
}

void batchAnalysis2Thread::acquisitionSequence(){
    name = seq +"_"+ alg;

    //foldername = "D:/"+seq;
    foldername = "D:/Acquisizioni_Nicoletta/320X240_30fps/"+seq+"Images/";
    numbername1 [50];
    number1= sprintf(numbername1, "%8.8d.ppm", k);
    filename1 = foldername+numbername1;
    //cout  << filename1 << endl;
    Matrix = cv::imread(filename1, CV_LOAD_IMAGE_COLOR);   // Read the file
    //Matrix = cv::imread("C:/Users/AVignolo/Desktop/acquisizioni_reaching_robot/reaching_1/leftDumper/00000000.ppm", CV_LOAD_IMAGE_COLOR);
    
    //cv::resize(TMP, It, cv::Size(TMP.cols*PARAMS.scale, TMP.rows*PARAMS.scale), cv::INTER_LINEAR );
    //resize(TMP, Matrix, cv::Size(320,240), cv::INTER_LINEAR );

    //Size size(320,240);//the dst image size,e.g.100x100
    //Mat dst;//dst image
    //Mat src;//src image
    //resize(src,dst,size);//resize image
    //Matrix=dst;

    //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    //imshow( "Display window", image1 );                   // Show our image inside it.

    //waitKey(1);
}



void batchAnalysis2Thread::motionToColor(cv::Mat U, cv::Mat V, cv::Mat& colorcodeMatrix){
    double minval_x, maxval_x, minval_y, maxval_y;
    cv::Point  minLoc_x, maxLoc_x, minLoc_y, maxLoc_y;
    cv::minMaxLoc(U,&minval_x, &maxval_x, &minLoc_x, &maxLoc_x);
    //std::cout  << "maxval U     "<< maxval_x << std::endl;
    cv::minMaxLoc(V,&minval_y, &maxval_y, &minLoc_y, &maxLoc_y);
    //std::cout  << "maxval V     "<< maxval_y << std::endl;

    uchar *ccMatrixPointer = colorcodeMatrix.data;                  //data is pointing to the Red of the 1st pixel

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float fx = U.at<float>(y, x);//motim.Pixel(x, y, 0);
            float fy = V.at<float>(y, x);//motim.Pixel(x, y, 1);
            //.Pixel(x, y, 0);
            float fxnorm, fynorm;

            if (maxval_x == minval_x)
                fxnorm=0.00000001;
            else
                fxnorm = (fx-minval_x)/(maxval_x-minval_x);
            if (maxval_y == minval_y)
                fynorm=0.00000001;
            else
                fynorm = (fy-minval_y)/(maxval_y-minval_y); 

            assert(fxnorm >=0); assert(fxnorm <= 1.00);
            assert(fynorm >= 0); assert(fynorm <= 1.00);
            
            //computeColor(fxnorm, fynorm, ccMatrixPointer);

            computeColor(fx, fy, ccMatrixPointer);   //fx and fy are not normalized, therefore they are not between 0 and 1
            ccMatrixPointer += 3;
        }
    }
}


void batchAnalysis2Thread::fakethresholding(cv::Mat& U, cv::Mat& V){
    printf("faketh start \n");
    U = cv::Mat::zeros(U.rows, U.cols, CV_32FC1);
    V = cv::Mat::zeros(V.rows, V.cols, CV_32FC1);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {         
                U.at<float>(y, x)=0.00010;
                V.at<float>(y, x)=-1.0;
        }
    }
    printf("faketh end \n");
}


void batchAnalysis2Thread::thresholding(cv::Mat& Ut, cv::Mat& Vt, cv::Mat& maskThresholding){
    cv::Mat MAGt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat THETAt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);

    // from u-v to Magnitude-Theta
    double minval, maxval;
    cv::Point  minLoc, maxLoc;
    cv::minMaxLoc(Ut,&minval, &maxval, &minLoc, &maxLoc);
    //std::cout  << minval << "Ut " << maxval << std::endl;
    cv::cartToPolar(Ut, Vt, MAGt, THETAt, false);
  
    cv::Mat Probt = cv::Mat::zeros(MAGt.rows, MAGt.cols, CV_32FC1);
    int DELTA = 10;
    int LATO = DELTA*2+1;
    for(int i = DELTA; i < Probt.rows-DELTA; ++i) {
        for(int j = DELTA; j < Probt.cols-DELTA; ++j) {
            //std::cout <<  MAGt.at<float>(i,j) << " " << TH1_ << " " << std::endl;  
            if(MAGt.at<float>(i,j)> TH1_ ) {
                cv::Mat Q = MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1));
                cv::Mat MQ = Q >= TH2_;	   //>= returns a map of 0 and 255 instead of 1
                MQ = MQ/255.;
                Probt.at<float>(i,j) = cv::sum(MQ).val[0]/((float)(LATO*LATO));		  // divide by lato*lato in such a way to have 1 as maximum
                if(Probt.at<float>(i,j) >= PTH_) {
                    Maskt.at<float>(i,j) = 1.0;
                }
                else {
                    Maskt.at<float>(i,j) =  0.0;
                } 
                //Maskt.at<float>(i,j) = (Probt.at<float>(i,j) >= PTH_)/255.;
                //Probt.at<float>(i,j) = ((float)(cv::sum(MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2_).val[0]))/((float)(LATO*LATO));
        
                Q.release();
                MQ.release();
            }
            else {
                if(Probt.at<float>(i,j) >= PTH_) {
                    Maskt.at<float>(i,j) = 1;  //ccc     1.0;
                }
                else {
                    Maskt.at<float>(i,j) =  0;  ///ccc 0.0
                }
            }
        }
    }


    //double minValt, maxValt;
    //cv::Point maxPost;
    //cv::minMaxLoc(MAGt, &minValt, &maxValt, NULL, &maxPost);

    //cv::Mat Maskt = MAGt >= TH_; // identificazione della roi
    //Maskt = (Probt >= PTH_)/255.;

    bool COND = cv::sum(Maskt).val[0] >= 0.01*(Ut.rows*Ut.cols);	//global statistics: we consider just what has a movement greater than the 1%  of  the total of the image
    //bool COND = cv::sum(Maskt).val[0] >= 0.01*(255); 
    //   std::cout << cv::sum(Maskt).val[0] << " " << 0.1*(Ut.rows*Ut.cols) << " " << COND << std::endl;


    /*
    if(!COND) {  
        //if(DEBUG_) {
        //  cv::Mat Im = cv::Mat::zeros(Ut.rows, Ut.cols, CV_8UC1);
        //  cv::imshow("DEBUG", Im);
        //  cv::waitKey(10);
        //  Im.release();
        //  
        //}
        MAGt.release();
        THETAt.release();
        MAGt_1.release();
        THETAt_1.release();
        Maskt.release();
        //computed = 0;                                   
        //return;               // REMEMBER: this makes it exit from the function
    }
    */
    
    
    //identify the connected components and visualize them
    //yInfo("identifying the connect components");
    std::vector<std::vector<cv::Point> > contours;

    //cv::Mat MaskClone = Maskt.clone();
    /*
            for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                cout <<  MaskClone.at<float>(y, x) <<  " " ;
        }
     }
     */
    cv::Mat helpframe;
    Maskt.convertTo(helpframe, CV_8U);
    
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(helpframe, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);	//MaskClone because findContours admit to write on the input
    //MaskClone.release();
  
    
    //yInfo("iterating the pixels");
    for(std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ) {						 //contours.begin() is the pointer to the initial element
        if(cv::contourArea(*it)/(Maskt.rows*Maskt.cols)>0.005)  //??? why 0.005
            ++it; //we are at the level of each bounding box,then the threshold is less
        else
            it = contours.erase(it);
    }

    
    //if(DEBUG_) {
    //  cv::Mat Im = cv::Mat::zeros(Probt.rows, Probt.cols, CV_8UC1);
    //  Probt_norm.convertTo(Im, CV_8UC1);
    //  cv::Mat Im3 = cv::Mat::zeros(Probt.rows, Probt.cols, CV_8UC3);
    //  cv::cvtColor(Im, Im3, CV_GRAY2RGB);
    //  cv::drawContours(Im3, contours, -1, CV_RGB(255,255,0), -1);
    //  cv::imshow("MAP", Im3);
    //  cv::waitKey(10);
    //  Im.release();
    //  
    //}

        /*for (int y = DELTA; y < height; y++) {
            for (int x = DELTA; x < width; x++) {
                cout <<  Maskt.at<float>(y, x) <<  " " ;
        }
     }*/
    //yInfo("drawing contours");
    cv::drawContours(maskThresholding, contours, -1, CV_RGB(255,255,255), -1);			 //it puts zeros where the optical flow is low and 255 where the optical   flow is high             -1 vuol   dire che  devo  colarle tutti  i bounding box. l'altro -1 che devo colarli  pieno (controlla)    

    //yInfo("removing the UV in the mask");



    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (maskThresholding.at<float>(y, x)==0.0) {
                Ut.at<float>(y, x) = 0.000001;
                Vt.at<float>(y, x) = 0.000001;
                //Maskt.at<float>(y, x) = 0;      //uncomment if you want to plot with the plotterThread Maskt (with copyM)
            }
            else {
                //Maskt.at<float>(y, x) = 255;   //uncomment if you want to plot with the plotterThread Maskt (with copyM)
            }
        }
     }

////da quiii
//    name = seq +"_"+ alg;
//    foldername = "C:/Users/AVignolo/Desktop/Acquisizioni_iCub/"+seq+"/Images/";
//
//    number1= sprintf(numbername1, "%8.8d", k);
//    number2= sprintf(numbername2, "%8.8d", k+1);
//
//
//    resultnameJPG="C:/Users/AVignolo/Desktop/results/"+seq+"_"+alg+"/jpg/"+seq+"_"+alg+"_"+numbername1+".jpg";
//    resultnameXML="C:/Users/AVignolo/Desktop/results/"+seq+"_"+alg+"/"+seq+"_"+alg+"_"+numbername1+".xml";
//    imwrite( resultnameJPG, colorcodeMatrix );
//    imwrite( "C:/Users/AVignolo/Desktop/colorcodeMatrix.jpg", colorcodeMatrix );
//    cv::FileStorage file(resultnameXML, cv::FileStorage::WRITE);
//    //Write to file!
//    file << "colorcodeMatrix" << colorcodeMatrix;
//    waitKey(50000);
//


}



void batchAnalysis2Thread::thresholding(cv::Mat Ut_1, cv::Mat Vt_1, cv::Mat& Ut, cv::Mat& Vt, cv::Mat& maskThresholding, std::vector<float>& descr, int& computed){ 
    cv::Mat MAGt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat THETAt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat MAGt_1 = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat THETAt_1 = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);  

    int h=Ut_1.depth();
    int i=Ut.depth();
    // from u-v to Magnitude-Theta
    cv::cartToPolar(Ut, Vt, MAGt, THETAt, false);
  
    cv::Mat Probt = cv::Mat::zeros(MAGt.rows, MAGt.cols, CV_32FC1);
    int DELTA = 10;
    int LATO = DELTA*2+1;
    for(int i = DELTA; i < Probt.rows-DELTA; ++i) {
        for(int j = DELTA; j < Probt.cols-DELTA; ++j) {
            //std::cout <<  MAGt.at<float>(i,j) << " " << TH1_ << " " << std::endl;  
            if(MAGt.at<float>(i,j)> TH1_ ) {
                cv::Mat Q = MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1));
                cv::Mat MQ = Q >= TH2_;	   //>= returns a map of 0 and 255 instead of 1
                MQ = MQ/255.;
                Probt.at<float>(i,j) = cv::sum(MQ).val[0]/((float)(LATO*LATO));		  // divide by lato*lato in such a way to have 1 as maximum
                
                //Probt.at<float>(i,j) = ((float)(cv::sum(MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2_).val[0]))/((float)(LATO*LATO));
        
                Q.release();
                MQ.release();
            }
        }
    }
    
    

    // double minValt, maxValt;
    // cv::Point maxPost;
    // cv::minMaxLoc(MAGt, &minValt, &maxValt, NULL, &maxPost);
  
    //   cv::Mat Maskt = MAGt >= TH_; // identificazione della roi
    cv::Mat Maskt = (Probt >= PTH_)/255.;
    int n=Maskt.depth();
    
    
    bool COND = cv::sum(Maskt).val[0] >= 0.01*(Ut.rows*Ut.cols);	//global statistics: we consider just what has a movement greater than the 1%  of  the total of the image
    //bool COND = cv::sum(Maskt).val[0] >= 0.01*(255); 
    //   std::cout << cv::sum(Maskt).val[0] << " " << 0.1*(Ut.rows*Ut.cols) << " " << COND << std::endl;

    
    /*
    if(!COND) {  
        //if(DEBUG_) {
        //  cv::Mat Im = cv::Mat::zeros(Ut.rows, Ut.cols, CV_8UC1);
        //  cv::imshow("DEBUG", Im);
        //  cv::waitKey(10);
        //  Im.release();
        //  
        //}
        MAGt.release();
        THETAt.release();
        MAGt_1.release();
        THETAt_1.release();
        Maskt.release();
        //computed = 0;                                   
        //return;               // REMEMBER: this makes it exit from the function
    }
    */
    
    
    // identify the connected components and visualize them
    //yInfo("identifying the connect components");
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat MaskClone = Maskt.clone();
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(MaskClone, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);	//MaskClone because findContours admit to write on the input
    MaskClone.release();
  
    
    //yInfo("iterating the pixels");
    for(std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ) {						 //contours.begin() is the pointer to the initial element
        if(cv::contourArea(*it)/(Maskt.rows*Maskt.cols)>0.005)  //??? why 0.005
            ++it; //we are at the level of each bounding box,then the threshold is less
        else
            it = contours.erase(it);
    }

    
    //if(DEBUG_) {
    //  cv::Mat Im = cv::Mat::zeros(Probt.rows, Probt.cols, CV_8UC1);
    //  Probt_norm.convertTo(Im, CV_8UC1);
    //  cv::Mat Im3 = cv::Mat::zeros(Probt.rows, Probt.cols, CV_8UC3);
    //  cv::cvtColor(Im, Im3, CV_GRAY2RGB);
    //  cv::drawContours(Im3, contours, -1, CV_RGB(255,255,0), -1);
    //  cv::imshow("MAP", Im3);
    //  cv::waitKey(10);
    //  Im.release();
    //  
    //}

    //yInfo("drawing contours");
    cv::drawContours(maskThresholding, contours, -1, CV_RGB(255,255,255), -1);			 //it puts zeros where the optical flow is low and 255 where the optical   flow is high             -1 vuol   dire che  devo  colarle tutti  i bounding box. l'altro -1 che devo colarli  pieno (controlla)    

    //yInfo("removing the UV in the mask");
    for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
            if (maskThresholding.at<float>(y, x)==0.0) {
                Ut.at<float>(y, x) = 0.000001;
                Vt.at<float>(y, x) = 0.000001;
            }             
		}
    }
//}


//void batchAnalysis2Thread::computeFeatures(cv::Mat Ut_1, cv::Mat Vt_1, cv::Mat Ut, cv::Mat Vt, std::vector<float>& descr, int& computed){   //I eliminated CUM
    descr.clear();

    /*if(!COND) {
    
        if(DEBUG_) {
          cv::Mat Im = cv::Mat::zeros(Ut.rows, Ut.cols, CV_8UC1);
          cv::imshow("DEBUG", Im);
          cv::waitKey(10);
          Im.release();
      
        }
    
        MAGt.release();
        THETAt.release();
        MAGt_1.release();
        THETAt_1.release();
        Maskt.release();
        computed = 0;
        return;
    }*/


    /*for(int x = 0; x < NewMask.cols; ++x) {
      for(int y = 0; y < NewMask.rows; ++y) {
	if(NewMask.at<uchar>(y,x) > 0) {
	  
	  CUM.at<float>(y,x) += MAGt.at<float>(y,x);   //CUM is just to  visualize the evolution of the region (it is something like to put together the binary matrices)->it can be commented

	  
	    }
      }
   }*/


  /*if(DEBUG_) {
    

    double minVal, maxVal;
    cv::Point maxPos;
    cv::minMaxLoc(CUM, &minVal, &maxVal, NULL, &maxPos, NewMask);
    
    cv::Mat Ig = cv::Mat::zeros(CUM.rows, CUM.cols, CV_8UC3);
    cv::Mat C = 255*(CUM-((float)(minVal)))/(((float)(maxVal))-((float)(minVal)));     //Normalization of CUM
    C.convertTo(Ig, CV_8UC3);
    
    cv::circle(Ig, maxPos, 3, CV_RGB(255,0,0));
    
    cv::imshow("DEBUG", NewMask);
    //cv::imshow("DEBUG", NewMask3);
    cv::waitKey(10);
    //If.release();
    //Im.release();
    //Imrgb.release();
    
    }*/

    cv::Mat VEL = cv::Mat::zeros(1,3,CV_32FC1), ACC = cv::Mat::zeros(1,3,CV_32FC1);
    cv::Moments MOMt;
    double minValt_1, maxValt_1;
    cv::Point maxPost_1;
    cv::Mat Probt_1;
    cv::Mat Maskt_1;
    cv::Mat mean, std;
    cv::Mat MatV = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat MatC = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat MatR = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat MatA = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    int delta = 3;

    //AAA togliere commenti e decidere dove settare il case, per ora c'e' solo quella del centroide
    //switch(flag_) {     //dove lo setto
    //case 0: // analisi del centroide  

    // devo trovare il centroide della roi         //roi=region of interest
    MOMt = cv::moments(Maskt, true);    //moments(Calculates all of the moments up to the third order of a polygon or rasterized shape.)
    cv::Point maxPost;
    if (MOMt.m00 == 0)
        MOMt.m00 = 0.000001;
    maxPost.x = MOMt.m10/MOMt.m00;
    maxPost.y = MOMt.m01/MOMt.m00;
    // spatial moments
    //double  m00, m10, m01, m20, m11, m02, m30, m21, m12, m03;
    // central moments
    //double  mu20, mu11, mu02, mu30, mu21, mu12, mu03;
    // central normalized moments
    //double  nu20, nu11, nu02, nu30, nu21, nu12, nu03;

    cv::cartToPolar(Ut_1, Vt_1, MAGt_1, THETAt_1, false);

    Probt_1 = cv::Mat::zeros(MAGt_1.rows, MAGt_1.cols, CV_32FC1);

      for(int i = DELTA; i < Probt_1.rows-DELTA; ++i) {        //as  before  during segmentation -> Probt_1 will be a matrix of values of pVbilities between 0 and 1 
        for(int j = DELTA; j < Probt_1.cols-DELTA; ++j) {
            if(MAGt_1.at<float>(i,j)> TH1_)
                Probt_1.at<float>(i,j) = ((float)(cv::sum(MAGt_1(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2_).val[0]))/((float)(LATO*LATO));
        }
      }
      
      Maskt_1 = Probt_1 >= PTH_;

    COND = cv::sum(Maskt_1).val[0] >= 0.05*(255*Ut.rows*Ut.cols);   //?rimane sempre a 0, quindi  ho  tolto l'if successivo  
    //if(COND) {
	//  yInfo("interno del  cond");
    //}

	//    devo trovare il centroide della roi
	cv::Moments MOMt_1 = cv::moments(Maskt_1, true);
    if (MOMt_1.m00 == 0) {
        MOMt_1.m00 = 0.000001;
    }
	maxPost_1.x = MOMt_1.m10/MOMt_1.m00;      //??
	maxPost_1.y = MOMt_1.m01/MOMt_1.m00;

	// DESCRITTORE: Vt Ct Rt At

    float u=Ut.depth();
    float v=Vt.depth();
    float z=Maskt.depth();
	
    //Maskt.convertTo(Maskt8U, CV_8U);
	VEL.at<float>(0,0) = cv::mean(Ut, Maskt).val[0]; 
	VEL.at<float>(0,1) = cv::mean(Vt, Maskt).val[0]; 
	VEL.at<float>(0,2) = 1.0/10.0;  //10 fps
	
    descr.push_back(sequenceID);
    timeCounter++;
    descr.push_back(timeCounter);

    float V =  sqrt(VEL.at<float>(0,0) * VEL.at<float>(0,0) + VEL.at<float>(0,1) * VEL.at<float>(0,1) + VEL.at<float>(0,2)*VEL.at<float>(0,2))  ;
	descr.push_back(V);         //V  is the norm of


	ACC.at<float>(0,0) =  cv::mean(Ut, Maskt).val[0] - cv::mean(Ut_1, Maskt_1).val[0]; 
	ACC.at<float>(0,1) = cv::mean(Vt, Maskt).val[0] - cv::mean(Vt_1, Maskt_1).val[0]; 
	ACC.at<float>(0,2) = 0.;
	
	descr.push_back(cv::norm(VEL.cross(ACC))/pow(cv::norm(VEL),3));
	descr.push_back(1/descr[1]);
	descr.push_back(descr[0]/descr[2]);
	
	descr.push_back(maxPost.x);
	descr.push_back(maxPost.y);


    std::cout  << "  Ut Vt  " <<  cv::mean(Ut, Maskt).val[0] << " " <<  cv::mean(Vt, Maskt).val[0] << " "  << std::endl;  

    std::cout  << "Descriptor    " <<  descr[0] << " " << descr[1] << " " << descr[2] << " " << descr[3]  << std::endl;  
	computed = 1;

    Bottle b;

    b.add(descr[0]);           
    b.add(descr[1]);           
    b.add(descr[2]);           
    b.add(descr[3]);           
    b.add(descr[4]);           
    b.add(descr[5]);           



    //  } else {
	//computed = 0;
    //  }

}



bool batchAnalysis2Thread::processing(){
	//yDebug("processing");
    numberProcessing++;
    //std::cout  << "numberProcessing    " << numberProcessing << " " << std::endl;
	//cv::Mat Matrix((IplImage*) inputImage->getIplImage(), false);
    
	//cv::Mat outputMatrix((IplImage*) inputImage->getIplImage(), false);
	//cv::Mat currentMatrix;
	if(!firstProcessing) {
		previousMatrix=currentMatrix.clone();
	}
    int a  =  Matrix.depth();
    int b  =  currentMatrix.depth();
    
    cvtColor(Matrix, currentMatrix, CV_RGB2GRAY);

	
	//--------------------------------------------------------------------------

	if(!firstProcessing){

		cv::Mat flow;
		//U = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);  //I have already use U and V to put them in U_1 and V_1  //?or is it better to initialize U, V, U_1, V_1 in the threadInit?
        //V = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);

        cv::Mat U_1 = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
        cv::Mat V_1 = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);

        cv::Mat maskThresholding = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);   //??prima CV_32FC1
		cv::Mat MV[2]={U,V};
        std::vector<float> descr;   //length=4
        int computed=0;

        if(numberProcessing>=3){
            U_1=U.clone();
            V_1=V.clone();                
        }

		switch (ofAlgo) {
		case ALGO_FB:{
                double timeStartOF = Time::now();
				cv::calcOpticalFlowFarneback(previousMatrix, currentMatrix, flow, 0.2, 3, 19, 10, 7, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
               // int  deppp=previousMatrix.depth();
                double timeStopOF = Time::now();
                timeDiffOF =timeStopOF-timeStartOF;
                //yDebug("timeDiffOF %f", timeDiffOF);
                //cv::calcOpticalFlowFarneback(previousMatrix, currentMatrix, flow, 0.2, 3, 19, 10, 5, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
				cv::split(flow, MV);
		}break;

		case ALGO_LK:{
				yDebug("just entered in  Lk");
				std::vector<cv::Point2f> prevPts;
				std::vector<cv::Point2f> currPts;
				cv::Size winSize(5,5); 

				int delta_=1;
				vector<uchar> status;
				vector<float> err;
				yDebug("before the cycle for");
				for(int x = 0; x < previousMatrix.cols; x=x+delta_)	{
					for(int y = 0; y < previousMatrix.rows; y=y+delta_)	{
						prevPts.push_back(cv::Point2f((float)x,(float)y));				//in prevPts there are the xs and the ys of the image
					}
				}
				cv::calcOpticalFlowPyrLK(previousMatrix, currentMatrix, prevPts, currPts, status, err, winSize, 3);
				
				for(int k = 0; k < prevPts.size(); k++)	{  //I is a vector and each  element is a point (therefore with two fields)
					//std::cout  << currPts.at(k).x << " "  << currPts.at(k).y << std::endl;
					if (status.at(k))	{
						U.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  currPts.at(k).x - prevPts.at(k).x;
						V.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  currPts.at(k).y - prevPts.at(k).y;
					}
				}
				
				//flow.create(previousMatrix.rows, previousMatrix.cols, CV_32FC2);			  //create(rows,cols,type)					   in teoria qui e nelle prox 2 righe ci  va I.rows,I.cols  (ma devi usare size)
  
      
				//int P = 0;
				//for(int k = 0; k < I.size(); k++)	  
				//	if (status.at(k))	{
				//		U.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  I.at(k).x;
				//		V.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  I.at(k).y;
				//	}
				//  
				// cv::Mat mv[2] = {U, V};
				//cv::merge(mv, 2, flow);
      
				 prevPts.clear();
	  			 currPts.clear();
				 status.clear();
				 err.clear();
		}break;																							
		}  // switch optical  flow


		/* computing colorcode */
		cv::Mat colorcodeMatrix =  cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_8UC3);
		//motionToColor(U, V, colorcodeMatrix);
		////colorcodeMatrix.convertTo(outputMatrix,CV_8UC3);

        /*taking a point (x,y) with a flow magnitude more than a threshold (th1) and then 
        computing the number of points around the point (x,y) with  the magnitude of the flow more than a threshold (th2) and taking the number of points with a flow magnitude more than a threshold(th3)*/
        //fakethresholding(U, V);


	    //?metto qui?
        if(numberProcessing>=3) {        //you can compute the acceleration when you have 3 frames
            double timeStartThresholding =Time::now();

            thresholding(U, V, maskThresholding);

            double timeStopThresholdind =Time::now();
            timeDiffThresholding =timeStopThresholdind-timeStartThresholding;
            //yDebug("timeDiffThresholding %f", timeDiffThresholding);

            //thresholding(U_1, V_1, U, V, maskThresholding, descr, computed);
            //U = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
            //V = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
            motionToColor(U, V, colorcodeMatrix);            
            //computeFeatures(U_1, V_1, U, V, descr, computed);          //U_1 is the U at time t-1, U is the U at the time t  (sicuri??)
        }

		/* computing min and max */	
		/*
		double minval, maxval;	
		cv::Point  minLoc, maxLoc;
		cv::minMaxLoc(U,&minval, &maxval, &minLoc, &maxLoc);
		std::cout  << minval << " " << maxval << std::endl;
		cv::minMaxLoc(V,&minval, &maxval, &minLoc, &maxLoc);
		std::cout  << minval << " " << maxval << std::endl;
		*/		
				

		/* of magnitude visualization */
		/*
		cv::Mat M, MN;
		cv::sqrt(U.mul(U)+V.mul(V),M);
		//double minval, maxval;
		//cv::Point  minLoc, maxLoc;
		cv::minMaxLoc(M,&minval, &maxval, &minLoc, &maxLoc);
		std::cout  << minval << " " << maxval << std::endl;
		if(maxval>0.0){
			MN=255*(M-(float)minval)/((float)maxval-(float)minval);
		}
		else { 
			MN=cv::Mat::zeros(U.rows,U.cols, CV_32FC1);
		}
		MN.convertTo(outputMatrix,CV_8UC1);
		*/

		//IplImage Ipl=(IplImage)outputMatrix;
		IplImage Ipl = (IplImage) colorcodeMatrix; 
		processingImage-> wrapIplImage(&Ipl);
        if (outputPort.getOutputCount()) {
            V.convertTo(outputMatrix,CV_8UC1);              //it is plotted just V
            IplImage tempIpl = (IplImage) outputMatrix;
            outputImage-> wrapIplImage(&tempIpl);
            //outputImage-> wrapIplImage(&((IplImage)(outputMatrix)));
        }

		flow.release();
        maskThresholding.release();
		colorcodeMatrix.release();

        U_1.release();
		V_1.release();
		//M.release();			  //if the block /* of magnitude visualization */ is uncommented, then uncomment these 2 lines also
		//MN.release();
		
	} // if(processing!= firstProcessing)
	else {
        U = cv::Mat::zeros(height, width, CV_32FC1);  //I have already use U and V to put them in U_1 and V_1  //?or is it better to initialize U, V, U_1, V_1 in the threadInit?
        V = cv::Mat::zeros(height, width, CV_32FC1);

        Maskt = cv::Mat::zeros(height, width,  CV_32FC1);   //??prima   CV_32FC1
        Maskt_1 = cv::Mat::zeros(height, width,  CV_32FC1);   //??prima   CV_32FC1

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                U.at<float>(y, x) = 0.000001;
                V.at<float>(y, x) = 0.000001;
                //Maskt.at<float>(y, x) = 0.000001;
              }
         }
		firstProcessing=false;
	}
    //--------------------------------------------------------------------------
	Matrix.release();

    //zzz
        contentBottleTime = timeBottle.addList();

        contentBottleTime.addDouble(timeDiffOF);
        contentBottleTime.addDouble(timeDiffThresholding);
        contentBottleTime.addDouble(timeDiffProcessing);
    return true;
}

void batchAnalysis2Thread::threadRelease() {
    // nothing    
}

void batchAnalysis2Thread::onStop() {
    //delete inputImage;
    inputImage.release();
	delete outputImage;
	delete processingImage;
    U.release();
    V.release();
    if(pt!=NULL){
        //yDebug("stopping the plotter thread");
        //printf("stopping the plotter thread \n");
        pt->stop();
    }

    if(fet!=NULL){
        //yDebug("stopping the plotter thread");
        //printf("stopping the plotter thread \n");
        fet->stop();
    }
    yInfo("closing the ports");
    //printf("closing the ports \n");

	inputPort.interrupt();
	inputPort.close();

    outputPort.interrupt();   
    outputPort.close();

    outputPortTime.interrupt();      //zzz
    outputPortTime.close();   //zzz
}