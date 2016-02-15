/** 
 *
 * \defgroup icub_zdfModule zdfMod
 * @ingroup icub_logPolarAttentionSystem
 *
 * Receives the left and right images from the robot and segments objects that are located in the fovea. Performs marker-less pixel-wise segmentation of an object located in the fovea.The output is an image of the segmented object in grayscale and a difference of gausian segmentation.
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP
 * IPP
 *
 * \section parameters_sec Parameters
 * 
 * Command-line Parameters 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c zdfMod.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c zdfMod/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c zdfMod \n   
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c inPortLeft \c /imageLeft:i \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c inPortRight \c /imageRight:i \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortProb \c /imageProb:o \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortSeg \c /imageSeg:o \n  
 *   specifies the output port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortDog \c /imageDog:o \n  
 *   specifies the output port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortCog \c /imageCog:o \n  
 *   specifies the output port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - \c /zdfMod \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /zdfMod
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /zdfMod/imageLeft:i \n
 *  - \c /zdfMod/imageRight:i \n
 *
 * Output ports
 *
 *  - \c /zdfMod \n
 *    see above
 *
 *  - \c /zdfMod/imageProb:o \n
 *  - \c /zdfMod/imageSeg:o \n
 *  - \c /zdfMod/imageDog:o \n
 *  - \c /zdfMod/imageCog:o \n
 *
 * Port types
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelBgr> >   \c inPortLeft;  \n 
 * \c BufferedPort<ImageOf<PixelBgr> >   \c inPortRight; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortProb;\n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortSeg; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortDog; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortCog;          
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
 * \c zdfMod.ini  in \c $ICUB_ROOT/app/zdfMod/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10 and Debian Stable 
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>zdfMod --name zdfMod --context zdfMod/conf --from zdfMod.ini </tt>
 * 
 * \author 
 * 
 * Vadim Tikhanoff, Andrew Dankers
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * Please site this paper when refering to this module as algorithms used have been taken from:
 * 
 * An Experimental Comparison of Min-Cut/Max-Flow Algorithms
 *   for Energy Minimization in Vision.
 *   Yuri Boykov and Vladimir Kolmogorov.
 *   In IEEE Transactions on Pattern Analysis and Machine Intelligence (PAMI)
 *
 * This file can be edited at \c $ICUB_ROOT/contrib/src/logPolarAttentionSystem/src/zdfModule/src
 *
 */

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
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
  

#include "iCub/zdfMod.h" 
using namespace yarp::os;

int main(int argc, char * argv[])
{
    /* initialize yarp network */ 
    Network yarp;

    /* create the module */
    zdfMod module; 

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose( true );
    rf.setDefaultConfigFile( "zdfMod.ini" ); //overridden by --from parameter
    rf.setDefaultContext( "zdfApplication" );   //overridden by --context parameter
    rf.configure( argc, argv );
 
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    return 0;
}
