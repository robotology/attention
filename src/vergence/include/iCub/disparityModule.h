// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
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

#include <iCub/disparityProcessor.h>
#include <iCub/disparityTool.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
// yarp
#include <yarp/math/Math.h>
#include <yarp/sig/Image.h>

/** 
 *
 * \defgroup icub_disparityModule disparityModule
 * @ingroup icub_logpolarAttention
 *
 * This is a module that computes disparity correlation function
 * 
 * \image html visualAttentionBUTD.jpg 
 * 
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c disparityModule.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c disparityModule/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c disparityModule \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 * 
 * 
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /disparityModule \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /disparityModule
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /disparityModule/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /disparityModule \n
 *    see above
 *
 *  - \c /disparityModule/image:o \n
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myInputPort; \n 
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myOutputPort;       
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c disparityModule.ini  in \c $ICUB_ROOT/app/disparityModule/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/disparityModule/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>disparityModule --name disparityModule --context disparityModule/conf --from disparityModule.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/contrib/src/logpolarAttention/vergence/include/iCub/disparityModule.h
 * 
 *
 *  \section change_log CHANGE LOG
 * 24/02/2012 : changed the output of the command port                            author : Rea \n
 * 24/02/2012 : added a new port for absolute vergence angle                      author : Rea
 */
 




class disparityModule : public RFModule {

private:
    /**
    * flag that indicates when the initialisation has been carried out
    */
    bool init_flag;
    Port cmdPort;
    
    std::string moduleName, robotName, robotPortName, ctrlType;
    std::string outputPortName; 

    //property to get encoders 
    yarp::os::Property optionsHead, optionsTorso;
    yarp::dev::IEncoders *encHead, *encTorso;

    yarp::dev::PolyDriver *robotHead, *robotTorso;

    yarp::sig::Vector fb, zl, pl, dl, ml;

    yarp::sig::Vector _q;
    yarp::sig::Vector _it;
    yarp::sig::Vector _o;
    yarp::sig:: Vector _epx;
    yarp::sig::Vector _tmp;
    yarp::sig::Vector _tmpEl;

    int _nFrame;

    double leftMax, leftMin, rightMax, rightMin;

    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb > >    imageInLeft;  //left camera port
    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelRgb > >    imageInRight; //right camera port
    yarp::os::BufferedPort < yarp::sig::ImageOf<yarp::sig::PixelMono > >   histoOutPort; //output histogram

    bool needLeft, needRight;
    int imgNumb;
    float ratio;
    FILE *fout;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgInL;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgInR;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> Limg;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> Rimg;

    /**
    * processor in charge of measuring the disparity and creating the histogram
    */
    disparityProcessor* currentProcessor;

    shift_Struct maxes[4];

public:
	
    typedef enum { KIN_LEFT = 1, KIN_RIGHT = 2, KIN_LEFT_PERI = 3, KIN_RIGHT_PERI = 4 } __kinType;

    disparityModule();
    ~disparityModule();
    /**
    * function for initialization and configuration of the RFModule
    * @param rf resourceFinder reference
    */
    virtual bool configure( yarp::os::ResourceFinder &rf );
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

    bool respond( const Bottle& command, Bottle& reply ) {

        string helpMessage =  string( getName().c_str() ) + 
            "commands are: \n" +  
            "help \n" + 
            "quit \n" +
            "ctrl use either ctrlGaze or arbitrer";
        reply.clear(); 

        if ( command.get(0).asString() == "quit" )
            return false;     
        else if ( command.get(0).asString() == "help" ) {
            cout << helpMessage;
            reply.addString( "ok" ); 
        }
        else if (command.get(0).asString() == "sus"){
            cout << " sending suspend signal" << endl;
            currentProcessor->suspend();
        }
        else if (command.get(0).asString() == "res"){
            cout << "sending resume signal" << endl;
            currentProcessor->resume();
        }
        else {
            cout << "command not known - type help for more info" << endl;
        }
        return true;
    }
    
    ImageOf<PixelMono> histo;//output image of the histogram sent on the port
};
//make gcc happy
