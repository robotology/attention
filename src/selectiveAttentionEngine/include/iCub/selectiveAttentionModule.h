// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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
 * @file selectiveAttnetionModule.cpp
 * @brief Implementation of the module necessary for selective attention
 */

#ifndef _selectiveAttentionModule_H_
#define _selectiveAttentionModule_H_

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <highgui.h>


//within Project Include
#include <iCub/selectiveAttentionProcessor.h>

// general command vocab's
const int32_t COMMAND_VOCAB_HELP                 = yarp::os::createVocab('h','e','l','p');
const int32_t COMMAND_VOCAB_SET                  = yarp::os::createVocab('s','e','t');
const int32_t COMMAND_VOCAB_GET                  = yarp::os::createVocab('g','e','t');
const int32_t COMMAND_VOCAB_RUN                  = yarp::os::createVocab('r','u','n');
const int32_t COMMAND_VOCAB_SUSPEND              = yarp::os::createVocab('s','u','s');
const int32_t COMMAND_VOCAB_RESUME               = yarp::os::createVocab('r','e','s');
const int32_t COMMAND_VOCAB_IS                   = yarp::os::createVocab('i','s');
const int32_t COMMAND_VOCAB_FAILED               = yarp::os::createVocab('f','a','i','l');
const int32_t COMMAND_VOCAB_OK                   = yarp::os::createVocab('o','k');
const int32_t COMMAND_VOCAB_CHILD_COUNT          = yarp::os::createVocab('c','c');
const int32_t COMMAND_VOCAB_WEIGHT               = yarp::os::createVocab('w');
const int32_t COMMAND_VOCAB_CHILD_WEIGHT         = yarp::os::createVocab('c','w');
const int32_t COMMAND_VOCAB_CHILD_WEIGHTS        = yarp::os::createVocab('c','w','s');
const int32_t COMMAND_VOCAB_NAME                 = yarp::os::createVocab('s','1');
const int32_t COMMAND_VOCAB_CHILD_NAME           = yarp::os::createVocab('c','n');
const int32_t COMMAND_VOCAB_SALIENCE_THRESHOLD   = yarp::os::createVocab('t','h');
const int32_t COMMAND_VOCAB_NUM_BLUR_PASSES      = yarp::os::createVocab('s','2');

const int32_t COMMAND_VOCAB_KBU = yarp::os::createVocab('k','b','u');//weight of the bottom-up algorithm
const int32_t COMMAND_VOCAB_KTD = yarp::os::createVocab('k','t','d'); //weight of top-down algorithm
const int32_t COMMAND_VOCAB_RIN = yarp::os::createVocab('r','i','n'); //red intensity value
const int32_t COMMAND_VOCAB_GIN = yarp::os::createVocab('g','i','n'); //green intensity value
const int32_t COMMAND_VOCAB_BIN = yarp::os::createVocab('b','i','n'); //blue intensity value

// directional saliency filter vocab's
const int32_t COMMAND_VOCAB_DIRECTIONAL_NUM_DIRECTIONS        = yarp::os::createVocab('d','n','d');
const int32_t COMMAND_VOCAB_DIRECTIONAL_NUM_SCALES            = yarp::os::createVocab('d','n','s');
const int32_t COMMAND_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX       = yarp::os::createVocab('d','s','i');
const int32_t COMMAND_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX   = yarp::os::createVocab('d','d','i');
const int32_t COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES = yarp::os::createVocab('d','a','n','s');
const int32_t COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME  = yarp::os::createVocab('d','a','n');
//option for the set command
const int32_t COMMAND_VOCAB_LHAND = yarp::os::createVocab('l','h','a','n');
const int32_t COMMAND_VOCAB_TIME  = yarp::os::createVocab('t','i','m','e');
const int32_t COMMAND_VOCAB_K1= yarp::os::createVocab('k','1');
const int32_t COMMAND_VOCAB_K2    = yarp::os::createVocab('k','2');
const int32_t COMMAND_VOCAB_K3    = yarp::os::createVocab('k','3');
const int32_t COMMAND_VOCAB_K4    = yarp::os::createVocab('k','4');
const int32_t COMMAND_VOCAB_K5    = yarp::os::createVocab('k','5');
const int32_t COMMAND_VOCAB_K6    = yarp::os::createVocab('k','6');
const int32_t COMMAND_VOCAB_BU    = yarp::os::createVocab('b','u');
const int32_t COMMAND_VOCAB_TD    = yarp::os::createVocab('t','d');
const int32_t COMMAND_VOCAB_KC1   = yarp::os::createVocab('k','c','1');
const int32_t COMMAND_VOCAB_KC2   = yarp::os::createVocab('k','c','2');
const int32_t COMMAND_VOCAB_KMOT  = yarp::os::createVocab('k','m','o','t');
const int32_t COMMAND_VOCAB_DEF   = yarp::os::createVocab('d','e','f');
const int32_t COMMAND_VOCAB_MOT   = yarp::os::createVocab('m','o','t');
const int32_t COMMAND_VOCAB_INH   = yarp::os::createVocab('i','n','h');
const int32_t COMMAND_VOCAB_NINH  = yarp::os::createVocab('n','i','n','h');

/**
*
* @ingroup icub_logpolarAttention
 * \defgroup icub_selAttentionEngine selAttentionEngine
 * 
 * Module that combines the saliency map using winner-takes-all algorithm and selects the attentive region of interest.
 * 
 * 
 * \section intro_sec Description
 * Module that combines the saliency map using winner-takes-all algorithm and selected the attentive region of interest.
 * It is in charge of closing the first loop of self-reingorcement. In other words, it sends back commands that are used
 * in order to select the area of interest more precisely.
 * 
 * 
 * 
 * The module does:
 * -   process the input images as saliency maps
 * -   determine the winner-take-all saliency region
 * -   apply the Inhibition of return (todo)
 * -   send feedback to the preattentive section of the application in order to reinforce the stimulus coming from selected area
 * 
 * 
 * \image html visualAttentionTDBU.jpg
 * 
 * \section lib_sec Libraries
 * YARP
 * 
 * 
 * \section parameters_sec Parameters
 * -- name : name of the module and relative port name
 * -- ratethread : the period between successive calls to the run of the thread
 * -- angles
 * -- rings
 * -- xsize
 * -- ysize
 * -- overlap
 * -- cameraSelection
 * -- gazePerform
 * -- saccadicInterval
 * -- k1, k2, k3, k4, k5, k6
 * -- kmot
 * -- kc1
 * -- hueMap
 * -- satMap
 * -- earlyStage
 *  
 * \section portsa_sec Ports Accessed
 * none
 * 
 * 
 * \section portsc_sec Ports Created
 * Input ports:
 * - map1:i     : image coming from the 1st saliency map
 * - map2:i     : image coming from the 2nd saliency map
 * - map3:i     : image coming from the 3rd saliency map
 * - map4:i     : image coming from the 4th saliency map
 * - map5:i     : image coming from the 5th saliency map
 * - map6:i     : image coming from the 6th saliency map
 * - motion:i   : cartesian input image of the flow motion
 * - cart:i     : cartesian input image in the saliency
 * - inhiCart:i : cartesian image for the inhibition of return
 * - inhi:i     : logpolar image for inhibition of return
 * 
 * Outports:
 * - attention:o :     graphical output of the selected region of interest (winner-take-all)
 * - combination:o :   graphical output that shows the linear combination of different maps
 * - centroid:o :      port where the coordinate of the attention focus are sent 
 * - feedback:o :      port necessary to send back commands to preattentive processors
 * - cmd:o :           port where the command representing the gaze behaviour is sent
 * - wta:o :           port dedicated to the representation of the WTA before saccade performing 
 * 
 * InOut ports:
 * -<name>/cmd
 * - portionRequest     port for asking the correct portion of the mosaic based on the azimuth and elevation
 * 
 * Possible commands that this module is responsive to are:
 * - set def : set the coefficient to the default value
 * - set <int> time : set the timing costant between different saccades (default 3000- correspond to the conf file saccadic event)
 * - set <double> k1 : coefficient for the linear combination of map1
 * - set <double> k2 : coefficient for the linear combination of map2
 * - set <double> k3 : coefficient for the linear combination of map3
 * - set <double> k4 : coefficient for the linear combination of map4
 * - set <double> k5 : coefficient for the linear combination of map5
 * - set <double> k6 : coefficient for the linear combination of map6
 * - set bu <double> : weight of the bottom-up contribution
 * - set td <double> : weight of the top-down contribution
 * - set kmot : coefficient for the linear combination of motion detection
 * - set kc1 : coefficient for linear combination of cartesian images
 * - set lhan : forces to redirect saccades only where the hand is located
 * 
 * - earlyStage : set active the earlySTage asynchronous process
 * 
 * \section in_files_sec Input Data Files
 * none
 * 
 * \section out_data_sec Output Data Files
 * none
 *  
 * \section conf_file_sec Configuration Files
 * --from selectiveAttention.ini (see Parameters section for list of entries of this file)
 * 
 * \section howTo How To Use It
 * The module can be simply activated as long the first input logpolar map is connected to the module port map1:i.
 * This module performs linear combination of this map with the other logpolar-maps. An extra cartesian map is provided and a map
 * for motion can be used as well. The module extracts the winner take all and send a ocular motion event to either directly to the
 * iCubInterface or to the gazeArbiter. The gazeArbiter handles all the asynchronous ocular commands and performs them in a smoother sequence.
 * 
 * \section tested_os_sec Tested OS
 * Linux and Windows.
 * 
 * \section example_sec Example Instantiation of the Module
 * selectiveAttentionEngine --file selectiveAttentionEngine.ini 
 * 
 * 
 * \author Francesco Rea
 * 
 * Copyright (C) 2008 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at \c $ICUB_ROOT/contrib/src/logPolarAttention/src/selectingAttention/include/iCub/selectiveModule.h
 *  
 *
 * \section change_log CHANGE LOG
 * 
 * 20/12/10 : changed the name from kMotion to kMot in parsing process of the config file       author:Rea\n
 * 21/12/10 : set to zero every map in the initialization code of the thread                    author:Rea\n
 * 21/12/10 : immediate response to the stimulus when it is very strong                         authot:Rea\n
 * 28/12/10 : added polydrivers for the cartesian control (prioceptive information)             author:Rea\n
 * 28/12/10 : added robotname parameter configurable from config file                           author:Rea\n
 * 04/01/11 : added initialisation to zero values of motion and cartesian images                author:Rea\n
 * 04/01/11 : added saccadic event in direction of the left arm (when requested)                author:Rea\n
 * 04/01/11 : added an in-class function for coping images and extract relevant information     author:Rea\n
 * 05/01/11 : introduced the mechanism that configure which are the satMap and hueMap           author:Rea\n
 * 05/01/11 : changed the copy function of logpolar images from copy_8u_C1R to copy_C1R         author:Rea\n
 * 06/01/11 : moved the hand tracking conditional statement at the beginning of the cycle       author:Rea\n
 * 06/01/11 : removed the map 5 which was duplicated as value map and introduced skinmap        author:Rea\n
 * 08/03/11 : added the new bottle(template) to the communication to the objectProp             author:Rea\n
 * 06/06/11 : created a new ouput image composed of a gaussian around the WTA                   author:Rea\n
 * 07/06/11 : added the portion of the mosaic in order to depress the saliency mpa              author:Rea\n
 * 09/08/11 : changed the algorithm to provide different timing in the saccade selection        author:Rea\n
 * 09/08/11 : changed the names of the ports to better align it to the biology                  author:Rea\n
 * 11/08/11 : changed the visiting algorithm for the image                                      author:Rea\n
 * 25/08/11 : added flags to be compatible with the previous versions of the visual att.        author:Rea\n
 * 30/08/11 : setting the flag of the earlyStage using resource finder                          author:Rea\n
 * 05/09/11 : adding the habituation image for storing space-dependent behaviour to suppress    author:Rea\n
 * 05/10/11 : adding the fast reaction in saccade from early stage and enabled second stage     author:Rea\n
 * 10/10/11 : enhanced the response in logpolar to allow the presence of the response in cart.  author:Rea\n
 * 25/04/12 : introduced the asynchronous stimulus for motion                                   author:Rea\n
 * 27/04/12 : added the facilitation map in the final linear combination of the feature maps    author:Rea\n  
*/


class selectiveAttentionModule : public yarp::os::RFModule {
private:
    double parameterZ;          // z distance of the objects necessary for monocular gazing
    double k1;                  // coefficient map1
    double k2;                  // coefficient map2
    double k3;                  // coefficient map3
    double k4;                  // coefficient map4
    double k5;                  // coefficient map5
    double k6;                  // coefficient map6
    double kMotion;             // coefficient map Motion
    double kc1;                 // coefficient map cartesian1
    double kc2;                 // coefficient map cartesian2

    yarp::os::Port cmdPort;     // command port of the module
    int ct;                     // counter of the module
    int hueMap;                 // index of the map referencing hue
    int satMap;                 // index of the map referencing sat
    int width;                                      // width of the input image
    int height;                                     // height of the input image
    int rateThread;                                 // rateThread of the processor Thread
    int xSize,ySize, numberOfRings,numberOfAngles;  // parameters of the configuration file
    int saccadicInterval;                           // time interval between saccades
    double overlap;                                 // value of the overlap
    bool reinit_flag;                               // flag that indicates when the reinitiazation has already be done

    yarp::os::Semaphore mutex;                      // semaphore for the respond function

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImg;      // input image reference
    yarp::sig::ImageOf<yarp::sig::PixelMono> *tmp;          // temporary mono image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *tmp2;          // temporary rgb image
    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *map1Img;      // input image of the 1st map
    yarp::sig::ImageOf<yarp::sig::PixelMono> *map2Img;      // input image of the 2nd map
    yarp::sig::ImageOf<yarp::sig::PixelMono> *map3Img;      // input image of the 3rd map
    yarp::sig::ImageOf<yarp::sig::PixelMono> *map4Img;      // input image of the 4th map
    yarp::sig::ImageOf<yarp::sig::PixelMono> *map5Img;      // input image of the 5th map
    yarp::sig::ImageOf<yarp::sig::PixelMono> *map6Img;      // input image of the 6th map
    
    selectiveAttentionProcessor *currentProcessor;          // processor that controls the processing of the input image

    std::string moduleName;                                 // name of the module read from configuration 
    std::string camSelection;                               // name of the attentive subsystem read from configuration (left/right)
    std::string gazePerform;                                // flag for gazePerforming
    std::string robotName;                                  // name of the robot
    std::string robotPortName;                              // reference to the head of the robot

    /**
    * function that resets all the mode flags
    */
    void resetFlags();

public:
    /**
    *open the ports of the module
    *@param config configuration of the module
    */
    bool open(yarp::os::Searchable& config);

    /**
    * tryes to interrupt any communications or resource usage
    */
    bool interruptModule();

    /**
    * closes the modules and all its components
    */
    bool close();

    /**
    * active control of the Module
    */
    bool updateModule();

    /**
    * function for initialization and configuration of the RFModule
    * @param rf resourceFinder reference
    */
    virtual bool configure(yarp::os::ResourceFinder &rf);

    /**
    * set the attribute options of class Property
    *@param options of the current module
    */
    void setOptions(yarp::os::Property options);

    /**
    * function to reainitialise the attributes of the class
    * @param weight size of the input image
    * @param height size of the input image
    */
    void reinitialise(int weight, int height);

    /**
    * respond to command coming from the command port
    * @param command command received
    * @param reply bottle used as a reply
    */
    bool respond(const yarp::os::Bottle &command,yarp::os::Bottle &reply);

    /**
    * creates some objects necessary for the window
    */
    void createObjects();

    /**
    * sets the adjustments in the window
    */
    void setAdjs();

    /**
    * opens all the ports necessary for the module
    */
    bool openPorts();

    /**
    * closes all the ports opened when the module started
    */
    bool closePorts();

    /**
    * streams out data on ports
    */
    bool outPorts();

    /**
    * sets the module up
    */
    void setUp();

    /**
    * function that starts the selectiveAttentionProcessor
    * @return if correctly started
    */
    bool startselectiveAttentionProcessor();

    bool inputImage_flag;                       //flag that controls if the inputImage has been ever read
    bool init_flag;                             //check of the already happened initialisation
};

#endif //_selectiveAttentionModule_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
