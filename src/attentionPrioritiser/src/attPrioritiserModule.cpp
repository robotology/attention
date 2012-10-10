// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file attPrioritiserModule.cpp
 * @brief Implementation of the attPrioritiserModule (see header file).
 */

#include <iCub/attPrioritiserModule.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace attention::predictor;
using namespace attention::evaluator;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool attPrioritiserModule::configure(yarp::os::ResourceFinder &rf) {

    collector   = 0;
    prioritiser = 0;
    controller  = 0;

    if(rf.check("help")) {
        printf("HELP \n");
        printf("--name : changes the rootname of the module ports \n");
        printf("--robot : changes the name of the robot where the module interfaces to  \n");
        printf("--learningController : learning process for the controller activated \n");
        printf("--visualFeedback : indicates whether the tracker can be initialised \n");
        printf("--name : rootname for all the connection of the module \n");
        printf("====== \n");
        printf("press CTRL-C to continue.. \n");
        return true;
    }

    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName             = rf.check("name", 
                           Value("/attPrioritiser"), 
                           "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());
    printf("module will be activated with the name: %s \n", getName().c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";
    printf("robotName: %s \n", robotName.c_str());
    
    configName             = rf.check("config", 
                           Value("icubEyes.ini"), 
                           "Config file for intrinsic parameters (string)").asString();
    printf("configFile: %s \n", configName.c_str());

    if (strcmp(configName.c_str(),"")) {
        printf("looking for the config file \n");
        configFile=rf.findFile(configName.c_str());
        printf("config file %s \n", configFile.c_str());
        if (configFile=="") {
            printf("ERROR: file not found");
            return false;
        }
    }
    else {
        configFile.clear();
    }

    robotName             = rf.check("visualFeedback", 
                                     Value("icub"), 
                                     "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";
    printf("robotName: %s \n", robotName.c_str());
    
    //launching components
    printf("running the prioCollectorThread \n");
    collector=new prioCollectorThread();
    collector->setName(getName().c_str());
    
    printf("\n running the prioritiser \n");
    prioritiser=new attPrioritiserThread(configFile);
    prioritiser->setName(getName().c_str());
    prioritiser->setRobotName(robotName);

    printf("running the controller \n");
    controller = new oculomotorController(prioritiser);
    string str = (string) getName();
    str.append("/controller");
    controller->setName(str.c_str());
    controller->setResourceFinder(&rf);
    controller->setLogFile(rf.findFile("logFile.txt"));
    controller->setLogState(rf.findFile("logState.txt"));
    controller->setPsaFile(rf.findFile("psaFile.txt"));
    controller->setRewardFile(rf.findFile("rewardFile.txt"));
    controller->setQualityFile(rf.findFile("qualityFile.txt"));
    printf("controller correctly initialised \n");
    

    //controller->setResourceFinder(rf);
    //if (rf.check("visualFeedback")) {
    //    prioritiser->setVisualFeedback(true);
    //    printf("visualFeedback required \n");
    //}
    //else {
    //    printf("visualFeedback  not required \n");
        //the default value for prioritiser->visualCorrection is false
    //}

    /* get the dimension of the image for the thread parametric control */
    width                  = rf.check("width", 
                           Value(320), 
                           "width of the image (int)").asInt();

    height                 = rf.check("height", 
                           Value(240), 
                           "height of the image (int)").asInt();

    printf("\n width: %d  height:%d \n", width, height);
    prioritiser->setDimension(width,height);

    /* offset for 3d position along x axis */
    this->xoffset       = rf.check("xoffset", 
                           Value(0), 
                           "offset for 3D fixation point x").asDouble();
    printf("xoffset:%f \n", xoffset);
    prioritiser->setXOffset(xoffset);

    /* offset for 3d position along y axis */
    this->yoffset       = rf.check("yoffset", 
                           Value(0), 
                           "offset for 3D fixation point y").asDouble();
    printf("yoffset:%f \n", yoffset);
    prioritiser->setYOffset(yoffset);

    /* offset for 3d position along z axis */
    this->zoffset       = rf.check("zoffset", 
                           Value(0), 
                           "offset for 3D fixation point z").asDouble();
    printf("zoffset:%f \n", zoffset);
    prioritiser->setZOffset(zoffset);

    
    // limits for 3d position along x axis 
    xmax       = rf.check("xmax", 
                           Value(-0.2), 
                          "limit max for 3D fixation point x").asDouble();
    printf("xmax:%f \n", xmax);
    xmin       = rf.check("xmin", 
                           Value(-10.0), 
                           "limit min for 3D fixation point x").asDouble();;
    printf("xmin:%f \n", xmin);
    prioritiser->setXLimits(xmax,xmin);
    
    // limits for 3d position along y axis 
    ymax       = rf.check("ymax", 
                           Value(0.3), 
                           "limit max for 3D fixation point y").asDouble();
    printf("ymax:%f \n", ymax);
    ymin       = rf.check("ymin", 
                           Value(-0.3), 
                           "limit max for 3D fixation point y").asDouble();
    printf("ymin:%f \n", ymin);
    prioritiser->setYLimits(ymax,ymin);
    
    // limits for 3d position along z axis 
    zmax       = rf.check("zmax", 
                           Value(0.9), 
                           "limit max for 3D fixation point z").asDouble();
    printf("zmax:%f \n", zmax);
    zmin       = rf.check("zmin", 
                           Value(-0.3), 
                           "limit min for 3D fixation point z").asDouble();
    printf("zmin:%f \n", zmin);
    prioritiser->setZLimits(zmax,zmin);
    
    // specifies whether the camera is mounted on the head
    //onWings       = rf.check("onWings", 
    //                       Value(0), 
    //                       "indicates whether the camera is mounted on the head").asInt();
    //printf("onWings %d \n", onWings);
    //prioritiser->setOnWings(onWings);
    
    // specifies whether the camera is mounted on the head
    mode       = rf.check("mode", 
                           Value("standard"), 
                           "indicates mapping with which the image plane is moved").asString();
    printf("mode seleected: %s \n", mode.c_str());
    
    //if(!strcmp("onWings", mode.c_str())) {
    //    printf("onWings %d true \n", onWings);
    //    prioritiser->setOnWings(true);
    //}
    //else if(!strcmp("onDvs", mode.c_str())) {
    //    printf("onDvs true  \n");
    //    prioritiser->setOnDvs(true);
    //} 
       
    // fixating pitch
    pitch       = rf.check("blockPitch", 
                           Value(-1), 
                           "fixing the pitch to a desired angle").asDouble();
    printf("pitch:%f \n", pitch);
    prioritiser->setBlockPitch(pitch);

    // setting observer and observable interactions    
    collector->addObserver(*prioritiser);
    prioritiser->addObserver(*controller);

    
    /**
     * indicates whether the module owns a visual feedback for preditction
     */
    if(rf.check("visualFeedback")) {
        printf("The module can use visual feedback for prediction \n");
        prioritiser->setVisualFeedback(true);
        
    }
    else {
        printf("The module cannot use visual feedback for prediction \n");
        prioritiser->setVisualFeedback(false);
    }


    /**
     * defining if the action selection is Q-learning controller`s call
     */
    if(rf.check("learningController")) {
        printf("The Q-learning controller takes responsabilities for any selected action \n");
        prioritiser->setLearning(true);
        controller->setIdle(false);
    }
    else {
        printf("Q learning controller is not involved in the action selection decisions \n");
        prioritiser->setLearning(false);
        controller->setIdle(true);
    }

    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }
    attach(handlerPort);                  // attach to port
    

    /**
     * adding predictors to the attPrioritiserThread
     */
    /*
    printf("\n \n ------------------------------------------------- ADDING PREDICTORS --------------------------------- \n");
    linAccModel* modelB = new linAccModel();
    modelB->init(1.0);
    int rowA = modelB->getA().rows();
    int colA = modelB->getA().cols();
    Vector z0(rowA);
    Vector x0(rowA);
    x0.zero();z0.zero();
    Matrix P0(rowA,colA);
    printf("initialisation of P0 %d %d \n", rowA, colA);
    for (int i = 0; i < rowA; i++) {
        for (int j = 0; j < colA; j++) { 

            P0(i,j) += 0.01;
        }      
    }
    printf("modelB\n %s \n %s \n", modelB->getA().toString().c_str(),modelB->getB().toString().c_str());    
    printf("P0\n %s \n", P0.toString().c_str());    
    genPredModel* mB = dynamic_cast<genPredModel*>(modelB);
   
    printf(" creating eval thread \n");
    evalThread etB(*mB);
    etB.init(z0,x0,P0);
    printf("genPred model A \n %s \n",mB    ->getA().toString().c_str());
    printf("lin acc model A \n %s \n",modelB->getA().toString().c_str());
    printf("just initialised genPredModel %08X \n",&etB);
    etB.start();
    prioritiser->addEvalThread(&etB);
    printf("just started genPredModel %08X \n",&etB);
    printf("------------------------------------------------------------------------------------------- \n");
    */
    
    
    printf("starting the collector... \n");
    collector->start();
    
    printf("starting the controller... \n");
    controller->start();
    

    printf("starting the prioritiser... \n");
    if(!prioritiser->start()){
        printf("Att.prioritiser thread did not start. \n");
        return false;
    }
    else {
        printf("Att.prioritiser thread correctly start. \n");
    }
    


    
    printf("all the components correcly started \n");

    

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool attPrioritiserModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool attPrioritiserModule::close() {
    handlerPort.close();
    
    printf("stopping the controller \n");
    if (controller != 0) {
        controller->stop();
        //delete controller;
        printf("deleting controller \n");
    }
    Time::delay(5.0);
   
    //------------------------------------------
    //stopping threads
    printf("stopping threads \n ");
    if(0 != collector) {
        collector->stop();
    }
    if(0 != prioritiser) {
        prioritiser->stop();
    }
    //delete collector;
    delete prioritiser;
    //------------------------------------------



    printf("attPrioritiserModule::close:success in closing \n");
    return true;
}

bool attPrioritiserModule::respond(const Bottle& command, Bottle& reply) {
    bool ok = false;
    bool rec = false; // is the command recognized?

    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else if ((command.get(0).asString()=="sus") || (command.get(0).asString()=="\"sus\"")) {
        //prioritiser->waitMotionDone();
        prioritiser->suspend();
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="res" || command.get(0).asString()=="\"res\"" ) {
        prioritiser->resume();
        reply.addString("ok");
    }
    
    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("many");
            reply.addString("help");

            //reply.addString();
            reply.addString("set fn \t: general set command ");
            reply.addString("get fn \t: general get command ");
            //reply.addString();

            
            //reply.addString();
            reply.addString("seek red \t : looking for a red color object");
            reply.addString("seek rgb \t : looking for a general color object");
            reply.addString("sus  \t : suspending");
            reply.addString("res  \t : resuming");
            //reply.addString();


            ok = true;
        }
        break;
    case COMMAND_VOCAB_SUSPEND:
        rec = true;
        {
            prioritiser->suspend();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_STOP:
        rec = true;
        {
            prioritiser->suspend();
            prioritiser->resume();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_RESUME:
    rec = true;
        {
            prioritiser->resume();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_SEEK:
        rec = true;
        {
            prioritiser->suspend();
            prioritiser->seek(command);
            prioritiser->resume();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_FIX:
        rec = true;
        {
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_CENT:
                {
                    printf("Fixating in Center \n");
                    prioritiser->fixCenter(1000);
                }
                break;
            }
            ok = true;
        }
        break;
    default: {
                
    }
        break;    
    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);
    
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool attPrioritiserModule::updateModule() {
    return true;
}

double attPrioritiserModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

