// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file attPrioritiserThread.h
 * @brief Definition of a thread that every time constant decide the action to take and the state to move starting from the set of inputs
 * (see attPrioritiserModule.h).
 */

#ifndef _ATT_PRIORITISER_THREAD_H_
#define _ATT_PRIORITISER_THREAD_H_

#define COMMAND_VOCAB_K1                 VOCAB2('k','1')
#define COMMAND_VOCAB_K2                 VOCAB2('k','2')
#define COMMAND_VOCAB_K3                 VOCAB2('k','3')
#define COMMAND_VOCAB_K4                 VOCAB2('k','4')
#define COMMAND_VOCAB_K5                 VOCAB2('k','5')
#define COMMAND_VOCAB_K6                 VOCAB2('k','6')
#define COMMAND_VOCAB_BU                 VOCAB2('b','u')
#define COMMAND_VOCAB_P0                 VOCAB2('p','0')

#define COMMAND_VOCAB_45                 VOCAB3('o','4','5')
#define COMMAND_VOCAB_P45                VOCAB3('p','4','5')
#define COMMAND_VOCAB_N45                VOCAB3('n','4','5')
#define COMMAND_VOCAB_M45                VOCAB3('M','4','5')
#define COMMAND_VOCAB_P90                VOCAB3('p','9','0')
#define COMMAND_VOCAB_SUSPEND            VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME             VOCAB3('r','e','s')
#define COMMAND_VOCAB_INH                VOCAB3('i','n','h')
#define COMMAND_VOCAB_ACT                VOCAB3('a','c','t')

#define COMMAND_VOCAB_PRED               VOCAB4('p','r','e','d')
#define COMMAND_VOCAB_STAT               VOCAB4('s','t','a','t')
#define COMMAND_VOCAB_NINH               VOCAB4('n','i','n','h')
#define COMMAND_VOCAB_NULL               VOCAB4('n','u','l','l')
#define COMMAND_VOCAB_TIME               VOCAB4('t','i','m','e')
#define COMMAND_VOCAB_TRED               VOCAB4('t','r','e','d')
#define COMMAND_VOCAB_TBLU               VOCAB4('t','b','l','u')
#define COMMAND_VOCAB_TGRE               VOCAB4('t','g','r','e')
#define COMMAND_VOCAB_FRGB               VOCAB4('f','r','g','b')

#define COMMAND_VOCAB_RED                VOCAB3('r','e','d')
#define COMMAND_VOCAB_GRE                VOCAB3('g','r','e')
#define COMMAND_VOCAB_BLU                VOCAB3('b','l','u')
#define COMMAND_VOCAB_SET                VOCAB3('s','e','t')
#define COMMAND_VOCAB_WTD                VOCAB3('w','t','d')
#define COMMAND_VOCAB_WBU                VOCAB3('w','b','u')
#define COMMAND_VOCAB_RGB                VOCAB3('r','g','b')
#define COMMAND_VOCAB_GET                VOCAB3('g','e','t')
#define COMMAND_VOCAB_ORI                VOCAB3('o','r','i')
#define COMMAND_VOCAB_MAXDB              VOCAB3('M','d','b')           // maximum dimension of the blob drawn
#define COMMAND_VOCAB_MINDB              VOCAB3('m','d','b')           // minimum dimension of the blob drawn


#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/all.h>
#include <iostream>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/all.h>
#include <string>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/pids.h>

#include <cv.h>
#include <highgui.h>

//within project includes
//#include <iCub/trackerThread.h>
#include <iCub/observer.h>
#include <iCub/observable.h>
#include <iCub/sacPlannerThread.h>
#include <iCub/trajectoryPredictor.h>
#include <iCub/attention/predModels.h>
#include <iCub/attention/evalThread.h>

#define NUMSTATES 7 // number of states in the attPrioritiser
                    // any state corresponds to an action for the oculomotorController

/**
* thread that given a series of collected commands executes the most prioritised command, calling
* the correct interface function of the iKinGazeCtrl.
* // state 6 = anticipatory saccade 
* // state 5 = express saccade
* // state 4 = saccade
* // state 3 = smooth pursuit
* // state 2 = vergence
* // state 1 = wait
* // state 0 = null
*/

class attPrioritiserThread : public yarp::os::RateThread, public observer, public observable {
private:
    static const int numIter    = 10;       // number of iteractions
    
    int cUpdate;                            // counter of updates
    std::string name;                       // rootname of all the ports opened by this thread
    std::string robot;                      // name of the robot read by the ResourceFinder
    std::string configFile;                 // configuration file of cameras (LEFT RIGHT)
    std::string waitType;                   // typology of the waiting
    yarp::sig::Matrix stateTransition;      // matrix of the state transition; weights of the transition
    yarp::sig::Vector stateRequest;         // buffer of requests  (vergence, smooth pursuit, saccade)
    yarp::sig::Vector state;                // vector where just one element can be 1 indicating the state in which the system is
    yarp::sig::Vector allowedTransitions;   // vector of allowed transitions
    yarp::sig::Vector xFix;                 // fixation coordinates
    short numberState;                      // stores the number of the state in which the control can be
    
    int accomplFlag[NUMSTATES];             // series of flags representing any action accomplished
    bool stopVergence;                      // flag that inhibits the vergence command
    bool done;                              // flag set to true when an gaze action is completed
    bool executing;                         // flag that is set during the execution of motion
    bool allowStateRequest[NUMSTATES];      // vector of flags for allowing state request
    bool waitResponse[NUMSTATES];           // flag that indicates which response should be waiting
    bool firstNull;                         // flags that limits the number of null state messages
    //bool firstConsistencyCheck;           // boolean flag that check whether consistency happened
    //bool visualCorrection;                // boolean flag for allowing visual correction of the fine position
    //bool isOnWings;                       // flag that gives information on where the cameras are mounted
    //bool onDvs;                           // flag for remapping dvs location into standard dimension
    bool firstVergence;                     // flag that allows the inhibition for train of vergence commands 
    bool ver_accomplished;                  // flag that enables again visual feature extraction inhibition off
    bool sp_accomplished;                   // flag that indicates that the smooth pursuit is successfully working
    bool postSaccCorrection;                // flag that allows post saccadic corrections
    bool mono;                              // flag that indicates whether the saccade is mono or not
    bool firstVer;                          // flag check during the vergence that indicates whether eye correction comes after a monoSaccadic event
    bool pred_accomplished;                 // flag for prediction accomplished
    bool accomplished_flag;                 // flag for the accomplished vergence
    bool correcting;                        // flag that allows the test for correction to take place
    bool reinfFootprint;                    // flag that allows the reinforcement of the features of the desired object
    bool idleReinf;                         // flag that enable the reinforcFootprint mechanism 
    bool learning;                          // flag that allows the Q-learning controller to take decisions
    bool validAction;                       // flag that indicates when the action is valid
    bool isPendingCommand;                  // flag that indicates that an action command must be sent in the highlevel loop
    bool visualFeedback;                    // indicates when input signal for visual feedback is present

    int u,v;                                // values passed for saccades
    int* collectionLocation;                // collection of location for the center of gravity saccade
    int  originalContext;                   // original context for the gaze Controller
    int  width, height;                     // dimension of the image

    double time;                            // request of preparing time 
    double xObject,yObject,zObject;         // coordinates of the object 
    double zDistance;                       // estimated distance of the object from the eye
    double varDistance;                     // calculated distance of the object from the eye 
    double blockNeckPitchValue;             // value for blocking the pitch of the neck
    double xOffset;                         // offset for the 3D point along x
    double yOffset;                         // offset for the 3D point along y
    double zOffset;                         // offset for the 3D point along z
    double xmax, xmin;                      // limits in fixation point
    double ymax, ymin;                      // limits in fixation point
    double zmax, zmin;                      // limits in fixation point
    double Vx, Vy;                          // velocity profile for the smooth pursuit
    double predVx, predVy;                  // last predicted velocity profil
    double predXpos, predYpos, predZpos;    // predicted X, Y, Z position of the stimulus after minJerk action completation
    double predTime;                        // predicted time necessary to accomplished the tracking action
    double waitTime;                        // time to allow in the wait action
    double timeoutResponseStart;            // starting time measure for the wait response
    double predDistance;                    // predicted distance of the stimulus from the fovea
    
    double phi;                             // value passed for relative vergence from the maximum shift
    double phi2;                            // value passed for relative vergence from the second maximum shift
    double phi3;                            // value passed for relative vergence from the third maximum shift
    double phiTOT;                          // accumulator of increments of vergence angles
     
    double timeoutStart,timeoutStop;        // start and stop timing to avoid that saccadic event can stuck
    double timetotStart,timetotStop;        // start and stop timing for the complete fixation task
    double timeout;                         // actual timer of the saccadic action
    double timetot;                         // actual timer of the complete fixation task
    double startAction;                     // starting the time measure for the action
    double timing;                          // temporal extension of the ocular movement [s]
    double accuracy;                        // measure of how close is the attention to the stimulus
    double amplitude;                       // amplitude of the single component of the ocular movement [deg/submov]
    double frequency;                       // frequency of the action [submov/sec]

    unsigned char feedbackBlobRed;          // value returned from the feedback coming from blobFinder
    unsigned char feedbackBlobGreen;        // value returned from the feedback coming from blobFinder
    unsigned char feedbackBlobBlue;         // value returned from the feedback coming from blobFinder
    unsigned char feedbackOri0;             // value returned from the feedback coming from orientation/earlyVision
    unsigned char feedbackOri45;            // value returned from the feedback coming from orientation/earlyVision
    unsigned char feedbackOri90;            // value returned from the feedback coming from orientation/earlyVision
    unsigned char feedbackOriM45;           // value returned from the feedback coming from orientation/earlyVision 
    
    int template_size;                      // size of the template
    int search_size;                        // area over the search is performed
    const static int tColor  = 10000;
    const static int tColOri = 10000;
    const static int tNull   = 10000;       // infrasaccadic time for topdown options
    bool topDownState[4];                   // vector of topDown states
    double kNull[7];
    double kColor[7];                       // kValue for selection in color state
    double kColOri[7];                      // kValue for selection in color 'n' orientation state 
    yarp::os::Bottle bufCommand[NUMSTATES]; // buffer for the last actions arranged divided by type
    yarp::os::Bottle *pendingCommand;       // control command to send

    iCub::iKin::iCubEye *eyeL;
    iCub::iKin::iCubEye *eyeR;
    yarp::sig::Matrix *invPrjL, *invPrjR;   // inverse of prjection matrix
    yarp::sig::Matrix *PrjL, *PrjR;         // projection matrix

    CvRect  template_roi;                   // region of interest of the template
    CvRect  search_roi;                     // region of interest of the search
    CvPoint point;                          // point result of the search
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb> * imgLeftIn;                                // input image 3 channel
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imgRightIn;                               // input mono image
    yarp::sig::ImageOf<yarp::sig::PixelMono>* templateImage;                            // image for the segmented object of the zdf
    yarp::sig::ImageOf<yarp::sig::PixelMono>* inhibitionImage;                          // image for the inhibition of return
   
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb > > inLeftPort;       // input image port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > inRightPort;      // output image port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb > > templatePort;     // port for the segmented object of the zdf
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > inhibitionPort;   // port for the segm
    yarp::os::BufferedPort<yarp::os::Bottle> desiredTrackPort;                          // port sending the u,v position on the image plane for tracking
    yarp::os::BufferedPort<yarp::os::Bottle> trackPositionPort;                         // port for the tracked position
    yarp::os::BufferedPort<yarp::os::Bottle> directPort;                                // port sending the u,v position on the image plane for tracking
    yarp::os::BufferedPort<yarp::os::Bottle> outputPort;                                // port necessary to send the gaze command to the gazeArbiter
    yarp::os::BufferedPort<yarp::os::Bottle> timingPort;                                // port where the timing of the fixation point redeployment is sent
    yarp::os::BufferedPort<yarp::os::Bottle> facePort;                                  // port for low-level face control
    yarp::os::BufferedPort<yarp::os::Bottle> emoPort;                                   // port for emotionInterface
    yarp::os::BufferedPort<yarp::os::Bottle> energyPort;                                // input port for energy measure
    
    yarp::os::BufferedPort<yarp::os::Bottle> highLevelLoopPort;                         // port dedicated to the retroactive high level loop

    yarp::os::Port feedbackEarlyVision;             // port for feedback to the early vision component of attention
    yarp::os::Port feedbackSelective;               // port for feedback to the selective component of visual attention                              
    yarp::os::Port feedbackProtoObject;             // port for feedback to proto-object feature extractor                              

    yarp::os::Port feedbackPort;                    // port necessary to communicate the status of the system
    yarp::os::Port blobDatabasePort;                // port where the novel location in 3d space is sent
    yarp::os::Property optionsHead;
    yarp::os::Semaphore mutex;                      // semaphore on the resource stateRequest
    yarp::os::Semaphore mutexAcc;                   // semaphore on the accomplished flags
   
    yarp::dev::IGazeControl *igaze;                 // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
    yarp::dev::PolyDriver *polyTorso, *robotHead;   // polydriver for the control of the head
    yarp::dev::IEncoders *encTorso, *encHead;       // measure of the encoder  (head and torso)

    attention::evaluator::evalThread evalVel1;      // evaluation thread velocity 1
    attention::evaluator::evalThread evalVel2;      // evaluation thread velocity 2
    attention::evaluator::evalThread evalAcc1;      // evaluation thread acceleration 1
    attention::evaluator::evalThread evalAcc2;      // evaluation thread accelaration 2
    attention::evaluator::evalThread evalMJ1_T1;    // evaluation thread minJerk distance 1 - period 1
    attention::evaluator::evalThread evalMJ2_T1;    // evaluation thread minJerk distance 2 - period 1
    attention::evaluator::evalThread evalMJ1_T2;    // evaluation thread minJerk distance 1 - period 2
    attention::evaluator::evalThread evalMJ2_T2;    // evaluation thread minJerk distance 2 - period 2

    attention::evaluator::evalQueue* eQueue;        // queue of evaluation threads
    
    yarp::sig::Matrix zMeasure;                     // vector of measurements
    yarp::sig::Matrix uMeasure;                     // vector of the input values
    
    trackerThread*    tracker;                      // reference to the object in charge of tracking a tamplete surrounding a point
    sacPlannerThread    *sacPlanner;                // planner of saccadic movements (todo: make it a list of planners
    trajectoryPredictor *trajPredictor;             // predictor of the trajectory of a given stimulus
    yarp::os::ResourceFinder* rf;                   // resourceFinder for the pathfinder of other files
    FILE* PsaFile;                                  // file that contains the Probability of Transitions
    
public:
    /**
    * default constructor
    */
    attPrioritiserThread(std::string configFile);

    /**
     * destructor
     */
    ~attPrioritiserThread();

    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 
 
    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);

    /**
    * function that set the robotname for the ports to which the module connects
    * @param str robotname as a string
    */
    void setRobotName(std::string str);

    /**
    * function that sets the variable needed for image dimension
    * @param width set the dimension width of the input image
    * @param height set the dimension height of the input image
    */
    void setDimension(int width, int height);

    /**
     * function that allows the Q-learning controller to take decisions
     * @param value bool value true: Q-learning decides false: action selection regulated by priorities
     */
    void setLearning(bool value) { learning = value; };

    /**
     * function that returns 1 if the learning process is active
     * @param value bool value true: Q-learning decides false: action selection regulated by priorities
     */
    bool isLearning() { return learning; };
    
    /**
     * function set allowStateRequest flag enabling a particular subset of oculomotorActions
     * @param pos id reference to the oculomotor action to enable/disable
     * @param value true/false to enable/disable a particular oculomotor action
     */
    void setAllowStateRequest(int pos, int value); 

    /**
     * @brief function that passes the resource finder to the class for further use of files
     * @param resourceFinder reference to the object
     */
    void setResourceFinder(yarp::os::ResourceFinder* resourceFinder) {rf = resourceFinder; };

    /**
     * @brief function that sends a particular command to the serial port for facial express
     * @param command string to be sent to the serial port
     */
    void setFacialExpression(std::string command);

    /**
     * @brief function that sets if the waiting is of type anticipatory or fixation
     * @param type string representing the type of waiting ("ant", "fix")
     */
    void setWaitType(std::string type) {
        waitType = type;
    };

    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that defines what has to be done once any observeble interrupts
    * @param o observable that has just interrupted the observer
    * @param arg Bottle that has passed from the observable to the observer
    */
    void update(observable* o, yarp::os::Bottle * arg);
 
    /**
     * @brief return the point extracted by the tracker
     */
    void getPoint(CvPoint& p);

    /**
     * initialise the process of tracking with the coordinates of the initial point
     */
    void init(int x, int y);

    /**
     * function that performs tracking of a point and its surroundings
     */
    void sqDiff(CvPoint &minloc);

    /**
    * function that sets the value of the parameter xOffset
    */
    void setXOffset(double  value) { xOffset = value; };

    /**
    * function that sets the value of the parameter yOffset
    */
    void setYOffset(double  value) { yOffset = value; };

    /**
    * function that sets the value of the parameter zOffset
    */
    void setZOffset(double  value) { zOffset = value; };

    /**
    * function that sets the value of the parameter x limirs
    */
    void setXLimits(double max, double min) { xmax = max; xmin = min; };

    /**
    * function that sets the value of the parameter y limits
    */
    void setYLimits(double max,double  min) { ymax = max; ymin = min; };

    /**
    * function that sets the variable visualFeedback
    */
    void setVisualFeedback(double value) { visualFeedback = value; };

    /**
     * function that sets the value head pitch to which the head is blocked
     * @param blockPitch value of the blockPitch
     */
    void setBlockPitch(double blockPitch);

    /**
    * function that sets the value of the parameter z limits
    */
    void setZLimits(double max,double min)    { zmax = max; zmin = min; };

    /**
     * function that returns the status of this oculomotor actuator
     */
    void isPredict(bool& returned_flag)       { mutexAcc.wait(); returned_flag = true; mutexAcc.post(); };

    /**
     * function that returns the status of this oculomotor actuator
     */
    void isSaccade(bool& returned_flag)       { mutexAcc.wait(); returned_flag = accomplFlag[3]; mutexAcc.post(); };

    /**
     * function that returns the status of this oculomotor actuator
     */
    void isVergence(bool& returned_flag)      { mutexAcc.wait(); returned_flag = accomplFlag[1]; mutexAcc.post(); };

    /**
     * function that returns the status of this oculomotor actuator
     */
    void isSmoothPursuit(bool& returned_flag) { mutexAcc.wait(); returned_flag = accomplFlag[2]; mutexAcc.post(); };

    /**
     * function that returns the status of this oculomotor actuator
     */
    void isAnticip(bool& returned_flag)       { mutexAcc.wait(); returned_flag = accomplFlag[5]; mutexAcc.post(); };
    
    /**
     * function that returns the status of the oculomotor actuator
     */
    void isValidAction(bool& returned_flag)       { mutexAcc.wait(); returned_flag = validAction; mutexAcc.post(); };
 
    /**
     * sets the value of the validAction flag
     */
    void setValidAction(bool value) {mutexAcc.wait(); validAction = value; mutexAcc.post();};

    /**
     * @brief function that send a color command to the preattentive stage
     * @redValue red component of the color
     * @greenValue green component of the color
     * @blueValue blue component of the color
     */
    void sendColorCommand(int redValue, int greenValue, int blueValue);


    /**
     * @brief function that send a action command that has to be performed via highLevelLoop
     */
    void sendPendingCommand();

    /**
     * @brief function that activates a seeking action
     * @param bot bottle that contains the seek tag, the typology of the search and some params
     */
    void seek(yarp::os::Bottle bot);
    
    /**
     * @brief function that force the gaze to be deployed in the center
     * @param time allowed after the saccade in fixation in ms
     */
    void fixCenter(int elapseTime);

    /**
    * function that returns only when the last action is ended
    */
    void waitMotionDone() {
        igaze->waitMotionDone();
    }

    /**
    * function that returns only when the corrective angle is ready for the sacPlannerThread
    * @return -1 only if correction is not necessary otherwise send the angle in degrees back 
    */
    int waitCorrection();
    
    /**
     * function that extracts the feature of the object in fovea to command feedback
     */
    void reinforceFootprint();

    /**
     * function that returns command saved in the buffer
     */
    yarp::os::Bottle getCommandBuffer(int pos) {return bufCommand[pos]; };

    /**
     * function printCommandBuffer prints all the commands
     */
    void printCommandBuffer();
    
    /**
     * function that returns when the gazeController is listening at the command port
     */
    bool readyForActions() {return outputPort.getOutputCount();};

    /**
     * @brief function that executes one particular action in the buffer of commands
     */
    bool executeCommandBuffer(int pos);

    /**
     * @brief function that executes one requested action event when not present in the buffer 
     */
    void executeClone(int pos);
    
    /**
     *  @brief function that adds all the evalThread to the attPrioritiser
     */
    void addCollectionEvalThread();

    /**
     * @brief function that adds evalThread to the list
     * @param et evalThread reference to evaluator to add
     */
    void addEvalThread(attention::evaluator::evalThread* et){
        printf(">>>>>>>>>>>>>>>>attPrioritiser::addEvalThread %08X \n", et);
        Vector x = et->getX();
        printf("attPRioritiser::addEvalThread: x = \n %s \n", x.toString().c_str());
        eQueue->push_back(et);
    };
};

#endif  //_ATT_PRIORITISER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

