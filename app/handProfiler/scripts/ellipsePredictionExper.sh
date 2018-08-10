
#######################################################################################
# USEFUL FUNCTIONS:                                                                  #
#######################################################################################
usage() {
cat << EOF
***************************************************************************************
DEA SCRIPTING
Author:  Alessandro Roncone   <alessandro.roncone@iit.it>

This script scripts through the commands available for the DeA Kids videos.

USAGE:
        $0 options

***************************************************************************************
OPTIONS:

*********USAGE:
******************************************************************************
EXAMPLE
***************************************************************************************
EOF
}

#######################################################################################
# FUNCTIONS:                                                                         #
#######################################################################################


####################################################################################################################
#                              Palm orientations                                                                   #
####################################################################################################################
palm4stick() {
    echo "PALM CUS (0.140137 0.654117 -0.743298 2.849317) " | yarp rpc /handProfiler
}

palmup() {
    echo "PALM CUS (-0.254182	 0.891684	-0.374555	 3.025273) " | yarp rpc /handProfiler
}

palmface() {
    echo "PALM CUS (-0.617941	 0.243286	-0.747637	 3.095665) " | yarp rpc /handProfiler
}

palmfaceright(){
   echo "PALM CUS (-0.037738	-0.996767	 0.070937	 2.145385)" | yarp rpc /handProfiler
}





####################################################################################################################
#                              Ellipses/Circle Trajectories                                                        #
####################################################################################################################

##################################### TTL
######## original full trajectories TTL

fastFullEllipseTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.018 0.33)))" | yarp rpc /handProfiler
}

mediumFullEllipseTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.015 0.33)))" | yarp rpc /handProfiler
}

slowFullEllipseTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.012 0.33)))" | yarp rpc /handProfiler
}

fastFullCircleTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.02 0.33)))" | yarp rpc /handProfiler
}

mediumFullCircleTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.017 0.33)))" | yarp rpc /handProfiler
}

slowFullCircleTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.014 0.33)))" | yarp rpc /handProfiler
}

######## full trajectories shifted 5 cm from the original TTL

fastFullEllipseTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.018 0.33)))" | yarp rpc /handProfiler
}

mediumFullEllipseTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.015 0.33)))" | yarp rpc /handProfiler
}

slowFullEllipseTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.012 0.33)))" | yarp rpc /handProfiler
}

fastFullCircleTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.02 0.33)))" | yarp rpc /handProfiler
}

mediumFullCircleTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.017 0.33)))" | yarp rpc /handProfiler
}

slowFullCircleTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.014 0.33)))" | yarp rpc /handProfiler
}

######## original partial trajectories TTL

fastPartialEllipseTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.018 0.33)))" | yarp rpc /handProfiler
}

mediumPartialEllipseTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.015 0.33)))" | yarp rpc /handProfiler
}

slowPartialEllipseTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.012 0.33)))" | yarp rpc /handProfiler
}

fastPartialCircleTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.02 0.33)))" | yarp rpc /handProfiler
}

mediumPartialCircleTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.017 0.33)))" | yarp rpc /handProfiler
}

slowPartialCircleTTL(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.014 0.33)))" | yarp rpc /handProfiler
}

######## partial trajectories shifted 5 cm from the original TTL

fastPartialEllipseTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.018 0.33)))" | yarp rpc /handProfiler
}

mediumPartialEllipseTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.015 0.33)))" | yarp rpc /handProfiler
}

slowPartialEllipseTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.012 0.33)))" | yarp rpc /handProfiler
}

fastPartialCircleTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.02 0.33)))" | yarp rpc /handProfiler
}

mediumPartialCircleTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.017 0.33)))" | yarp rpc /handProfiler
}

slowPartialCircleTTL05(){
    echo "GEN TTL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.014 0.33)))" | yarp rpc /handProfiler
}

##################################### CVP
######## original full trajectories CVP

fastFullEllipseCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.3)))" | yarp rpc /handProfiler
}

mediumFullEllipseCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.2)))" | yarp rpc /handProfiler
}

slowFullEllipseCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

fastFullCircleCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.3)))" | yarp rpc /handProfiler
}

mediumFullCircleCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.2)))" | yarp rpc /handProfiler
}

slowFullCircleCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

######## full trajectories shifted 5 cm from the original CVP

fastFullEllipseCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.3)))" | yarp rpc /handProfiler
}

mediumFullEllipseCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.2)))" | yarp rpc /handProfiler
}

slowFullEllipseCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

fastFullCircleCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.3)))" | yarp rpc /handProfiler
}

mediumFullCircleCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.2)))" | yarp rpc /handProfiler
}

slowFullCircleCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

######## original partial trajectories CVP

fastPartialEllipseCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.3)))" | yarp rpc /handProfiler
}

mediumPartialEllipseCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.2)))" | yarp rpc /handProfiler
}

slowPartialEllipseCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

fastPartialCircleCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.3)))" | yarp rpc /handProfiler
}

mediumPartialCircleCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.2)))" | yarp rpc /handProfiler
}

slowPartialCircleCVP(){
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

######## partial trajectories shifted 5 cm from the original CVP

fastPartialEllipseCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.3)))" | yarp rpc /handProfiler
}

mediumPartialEllipseCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.2)))" | yarp rpc /handProfiler
}

slowPartialEllipseCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

fastPartialCircleCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.3)))" | yarp rpc /handProfiler
}

mediumPartialCircleCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.2)))" | yarp rpc /handProfiler
}

slowPartialCircleCVP05(){
    echo "GEN CVP (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

##################################### TTPL
######## original full trajectories TTPL

fastFullEllipseTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.6 0.33)))" | yarp rpc /handProfiler
}

mediumFullEllipseTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

slowFullEllipseTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.25 0.33)))" | yarp rpc /handProfiler
}

fastFullCircleTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.6 0.33)))" | yarp rpc /handProfiler
}

mediumFullCircleTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

slowFullCircleTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.25 0.33)))" | yarp rpc /handProfiler
}

######## full trajectories shifted 5 cm from the original TTPL

fastFullEllipseTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.6 0.33)))" | yarp rpc /handProfiler
}

mediumFullEllipseTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

slowFullEllipseTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.15 0.2) (rev) (param 0.25 0.33)))" | yarp rpc /handProfiler
}

fastFullCircleTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.6 0.33)))" | yarp rpc /handProfiler
}

mediumFullCircleTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

slowFullCircleTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 4.71) (axes 0.2 0.2) (rev) (param 0.25 0.33)))" | yarp rpc /handProfiler
}

######## original partial trajectories TTPL

fastPartialEllipseTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.6 0.33)))" | yarp rpc /handProfiler
}

mediumPartialEllipseTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

slowPartialEllipseTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.25 0.33)))" | yarp rpc /handProfiler
}

fastPartialCircleTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.6 0.33)))" | yarp rpc /handProfiler
}

mediumPartialCircleTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

slowPartialCircleTTPL(){
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.25 0.33)))" | yarp rpc /handProfiler
}

######## partial trajectories shifted 5 cm from the original TTPL

fastPartialEllipseTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.6 0.33)))" | yarp rpc /handProfiler
}

mediumPartialEllipseTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

slowPartialEllipseTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.15 0.2) (rev) (param 0.25 0.33)))" | yarp rpc /handProfiler
}

fastPartialCircleTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.6 0.33)))" | yarp rpc /handProfiler
}

mediumPartialCircleTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

slowPartialCircleTTPL05(){
    echo "GEN TTPL (((O -0.3 -0.15 0.1) (A -0.3 0.05 0.1) (B -0.3 -0.15 0.3) (C -0.3 -0.15 -0.1) (theta -1.57 1.57 3.14) (axes 0.2 0.2) (rev) (param 0.25 0.33)))" | yarp rpc /handProfiler
}




####################################################################################################################
#                              Actions trajectories                                                                #
####################################################################################################################

########### right hand provide object

handRightP_TTLFront001_rev(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.2) (C -0.25 0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.01 0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPFront001_rev(){
    echo "GEN CVP (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.2) (C -0.25 0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.03)))" | yarp rpc /handProfiler
}

handRightP_TTLFront004_emp(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.3) (C -0.25 0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.04 0.33)))" | yarp rpc /handProfiler
}

######################################### right hand halt

handRightP_TTLHalt001_rev(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.2) (C -0.25 0.2 0.0) (theta 1.25 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.01 0.33)))" | yarp rpc /handProfiler
}

handRightP_TTLHalt002_emp(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.25) (C -0.2 0.2 0.0) (theta 1.25 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.04 0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPHalt008_emp(){
    echo "GEN CVP (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.25) (C -0.25 0.2 0.0) (theta 1.25 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.08)))" | yarp rpc /handProfiler
}

########################################### right hand pointing

handRightP_TTLPoint005_emp(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.35 0.2 -0.1) (B -0.25 0.2 0.35) (C -0.2 0.2 0.0) (theta 0.123 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.07 0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPPoint008_emp(){
    echo "GEN CVP (((O -0.25 0.2 0.1) (A -0.35 0.2 -0.1) (B -0.25 0.2 0.1) (C -0.25 0.2 0.0) (theta 0.125 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

handRightP_TTLPoint001_rev(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.35 0.2 -0.1) (B -0.25 0.2 0.2) (C -0.25 0.2 0.0) (theta 0.125 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.02 0.33)))" | yarp rpc /handProfiler
}


############################################ tests for Placing an object on a table

handRightP_TTLTable005_emp(){
    echo "GEN TTL (((O -0.2 0.2 0.0) (A -0.2 0.2 0.2) (B -0.25 0.2 0.10) (C -0.35 0.2 0.0) (theta 0.3 1.57 2.65) (axes 0.2 0.15) (param 0.1 0.33)))" | yarp rpc /handProfiler
}

handRightP_TTLTable002(){
    echo "GEN TTL (((O -0.2 0.2 0.0) (A -0.2 0.2 0.1) (B -0.25 0.2 0.1) (C -0.35 0.2 0.0) (theta 0.785 1.57 2.65) (axes 0.1 0.15) (param 0.03 0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPTable008_const(){
    echo "GEN CVP (((O -0.2 0.2 0.0) (A -0.15 0.2 0.1) (B -0.35 0.2 0.1) (C -0.35 0.2 0.0) (theta 0.785 1.57 2.65) (axes 0.1 0.2) (param 0.15)))" | yarp rpc /handProfiler
}


####################################################################################################################
#                              Linear Trajectories: Horizontal (left)                                              #
####################################################################################################################

handProfile_CVPHoriz005() {
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 -0.05 0.1) (B -0.4 -0.1 0.1) (C -0.3 -0.25 0.1) (theta 0.0 1.57 3.04) (rev) (axes 0.15 0.1) (param 0.05)))" | yarp rpc /handProfiler
}

handProfile_CVPHoriz01() {
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 -0.05 0.1) (B -0.4 -0.1 0.1) (C -0.3 -0.25 0.1) (theta 0.0 1.57 3.04) (rev) (axes 0.15 0.1) (param 0.1)))" | yarp rpc /handProfiler
}

handProfile_CVPHoriz02() {
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 -0.05 0.1) (B -0.4 -0.1 0.1) (C -0.3 -0.25 0.1) (theta 0.0 1.57 3.04) (rev) (axes 0.15 0.1) (param 0.2)))" | yarp rpc /handProfiler
}


handProfile_TTPLHoriz02() {
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.05 0.1) (B -0.4 -0.1 0.1) (C -0.3 -0.25 0.1) (theta 0.0 1.57 3.04) (rev) (axes 0.15 0.1) (param 0.2 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTPLHoriz04() {
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.05 0.1) (B -0.4 -0.1 0.1) (C -0.3 -0.25 0.1) (theta 0.0 1.57 3.04) (rev) (axes 0.15 0.1) (param 0.4 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTPLHoriz08() {
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.05 0.1) (B -0.4 -0.1 0.1) (C -0.3 -0.25 0.1) (theta 0.0 1.57 3.04) (rev) (axes 0.15 0.1) (param 0.8 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTPLHoriz12() {
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.05 0.1) (B -0.4 -0.1 0.1) (C -0.3 -0.25 0.1) (theta 0.0 1.57 3.04) (rev) (axes 0.15 0.1) (param 1.2 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTPLHoriz24() {
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.05 0.1) (B -0.4 -0.1 0.1) (C -0.3 -0.25 0.1) (theta 0.0 1.57 3.04) (rev) (axes 0.15 0.1) (param 2.4 0.33)))" | yarp rpc /handProfiler
}

handProfile_STAREXE() {
    echo "STAR EXE" | yarp rpc /handProfiler
}

handProfile_STARSIM() {
    echo "STAR SIM" | yarp rpc /handProfiler
}


#######################################################################################
#                                COMPLEX PROCEDURES                                   #
#######################################################################################




    ############################################################### left arm

    provideObject001() {
	palm4stick
	handProfile_TTLFront001_rev
        sleep 2.0 && handProfile_STAREXE
    }


    provideObjectEmp() {
	palm4stick
	handProfile_TTLFront004_emp
        sleep 2.0 && handProfile_STAREXE
    }

    provideObjectEmp_const(){
	palm4stick
	handRightP_CVPFront008_emp
        sleep 2.0 && handProfile_STAREXE
    }

    invite04() {
	palmface
	handProfile_TTLFront04_rev
        sleep 2.0 && handProfile_STAREXE
    }

    halt04(){
	palmface
	handProfile_TTLFront001_rev
        sleep 2.0 && handProfile_STAREXE
    }


    ################################################################ right arm

    provideObject001Right() {
	palm4stick
	handRightP_TTLFront001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    provideObject001Right_const() {
	palm4stick
	handRightP_CVPFront001_rev
        sleep 2.0 && handProfile_STAREXE
    }

     provideObjectEmpRight() {
	palm4stick
	handRightP_TTLFront004_emp
        sleep 2.0 && handProfile_STAREXE
    }

    haltRight04(){
	palmfaceright
	handRightP_TTLHalt001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    haltRight02Emp(){
	palmfaceright
	handRightP_TTLHalt002_emp
        sleep 2.0 && handProfile_STAREXE
    }

    haltRight08Const(){
	palmfaceright
	handRightP_CVPHalt008_emp
        sleep 2.0 && handProfile_STAREXE
    }

    pointRight04(){
	palm4stick
	handRightP_TTLPoint001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    pointRight05Emp(){
	palm4stick
	handRightP_TTLPoint005_emp
        sleep 2.0 && handProfile_STAREXE
    }

    pointRight08Const(){
	palm4stick
	handRightP_CVPPoint008_emp
        sleep 2.0 && handProfile_STAREXE
    }

############################ tests for table

    placeTableRight() {
	palm4stick
	handRightP_TTLTable002
        sleep 2.0 && handProfile_STAREXE
    }

    placeTableRightEmp() {
	palm4stick
	handRightP_TTLTable005_emp
        sleep 2.0 && handProfile_STAREXE
    }

    placeTableRightConst() {
	palm4stick
	handRightP_CVPTable008_const
        sleep 2.0 && handProfile_STAREXE
    }

########################### CTP tests

    pointRight04Ctp(){
	breathers "stop"
    	echo "ctpq time 1.5 off 0 pos (0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
	echo "ctpq time 1.5 off 0 pos (-8.4 43.4 13.6 97.3 -21.6 -8.4 24.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep 3.0
 	echo "ctpq time 1.5 off 0 pos (-3.0 0.0 5.0)" | yarp rpc /ctpservice/torso/rpc
	echo "ctpq time 1.5 off 0 pos (-36.9 25.7 23.0 14.5 14.4 -7.3 5.6 26.0 27.0 0.0 29.0 0.0 0.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	breathers "start"

    }

    haltRight04Ctp(){
	breathers "stop"
    	echo "ctpq time 1.5 off 0 pos (0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
	echo "ctpq time 1.5 off 0 pos (9.5 27.3 -4.2 75.5 40.0 -79.7 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep 3.0
 	echo "ctpq time 1.5 off 0 pos (-5.0 0.0 3.0)" | yarp rpc /ctpservice/torso/rpc
	echo "ctpq time 1.5 off 0 pos (-54.8 33.0 31.5 19.1 40.0 -79.7 -14.8 26.4 26.4 0.0 28.9 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	breathers "start"

    }

    provideObjectRight04Ctp(){
	breathers "stop"
    	echo "ctpq time 1.5 off 0 pos (0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
	echo "ctpq time 1.5 off 0 pos (9.4 27.3 -29.3 103.7 -42 24.1 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep 3.0
 	echo "ctpq time 1.5 off 0 pos (-3.0 0.0 3.0)" | yarp rpc /ctpservice/torso/rpc
	echo "ctpq time 1.5 off 0 pos (-49.5 62.7 34.6 68.2 -58.8 -7.3 24.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	breathers "start"

    }



#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
echo "********************************************************************************"
echo ""

$1 "$2"

if [[ $# -eq 0 ]] ; then
    echo "No options were passed!"
    echo ""
    usage
    exit 1
fi
