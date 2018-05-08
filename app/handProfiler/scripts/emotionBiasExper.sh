#!/bin/bash
#######################################################################################
# FLATWOKEN ICON THEME CONFIGURATION SCRIPT
# Copyright: (C) 2014 FlatWoken icons
# Author:  Alessandro Roncone
# email:   alecive87@gmail.com
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
#  *
# A copy of the license can be found at
# http://www.robotcub.org/icub/license/gpl.txt
#  *
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details
#######################################################################################


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

***************************************************************************************
EXAMPLE USAGE:

***************************************************************************************
EOF
}

#######################################################################################
# FUNCTIONS:                                                                         #
#######################################################################################
speak() {
    echo "\"$1\"" | yarp write ... /iSpeak
}

blink() {
    echo "blink" | yarp rpc /iCubBlinker/rpc
    sleep 0.5
}

breathers() {
    # echo "$1"  | yarp rpc /iCubBlinker/rpc
    echo "$1" | yarp rpc /iCubBreatherH/rpc:i
    echo "$1" | yarp rpc /iCubBreatherRA/rpc:i
    sleep 0.4
    echo "$1" | yarp rpc /iCubBreatherLA/rpc:i
}

breathersL() {
    echo "$1" | yarp rpc /iCubBreatherLA/rpc:i
}

breathersR() {
    echo "$1" | yarp rpc /iCubBreatherRA/rpc:i
}

head() {
    echo "$1" | yarp rpc /iCubBreatherH/rpc:i
}

stop_breathers() {
    breathers "stop"
}

start_breathers() {
    breathers "start"
}

go_home_withtable_helper() {
    # This is with the arms close to the legs
    # echo "ctpq time $1 off 0 pos (-6.0 23.0 25.0 29.0 -24.0 -3.0 -3.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/right_arm/rpc
    # echo "ctpq time $1 off 0 pos (-6.0 23.0 25.0 29.0 -24.0 -3.0 -3.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/left_arm/rpc
    # This is with the arms over the table
    go_home_withtable_helperR $1
    go_home_withtable_helperL $1
    # echo "ctpq time 1.0 off 0 pos (0.0 0.0 10.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
    go_home_helperH $1
    go_home_helperT $1
}

go_home_helper() {
    # This is with the arms close to the legs
    # echo "ctpq time $1 off 0 pos (-6.0 23.0 25.0 29.0 -24.0 -3.0 -3.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/right_arm/rpc
    # echo "ctpq time $1 off 0 pos (-6.0 23.0 25.0 29.0 -24.0 -3.0 -3.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/left_arm/rpc
    # This is with the arms over the table
    go_home_helperR $1
    go_home_helperL $1
    # echo "ctpq time 1.0 off 0 pos (0.0 0.0 10.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
    go_home_helperH $1
}

go_home_helperL()
{
    # echo "ctpq time $1 off 0 pos (-30.0 36.0 0.0 60.0 0.0 0.0 0.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time $1 off 0 pos (-6.0 23.0 25.0 29.0 -24.0 -3.0 -3.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/left_arm/rpc
}

go_home_withtable_helperL()
{
    # echo "ctpq time $1 off 0 pos (-30.0 36.0 0.0 60.0 0.0 0.0 0.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time $1 off 0 pos (-26.0 50.0 25.0 80.0 -24.0 -3.0 -3.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/left_arm/rpc
}

go_home_helperR()
{
    # echo "ctpq time $1 off 0 pos (-30.0 36.0 0.0 60.0 0.0 0.0 0.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time $1 off 0 pos (-6.0 23.0 25.0 29.0 -24.0 -3.0 -3.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/right_arm/rpc
}

go_home_withtable_helperR()
{
    # echo "ctpq time $1 off 0 pos (-30.0 36.0 0.0 60.0 0.0 0.0 0.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time $1 off 0 pos (-26.0 50.0 25.0 80.0 -24.0 -3.0 -3.0 19.0 29.0 8.0 30.0 32.0 42.0 50.0 50.0 114.0)" | yarp rpc /ctpservice/right_arm/rpc
}

go_home_helperH()
{
    echo "ctpq time $1 off 0 pos (-5.0 0.0 0.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
}

go_home_helperT()
{
    echo "ctpq time $1 off 0 pos (-3.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
}

go_homeH() {
    head "stop"
    go_home_helperH 1.5
    sleep 2.0
    head "start"
}

go_home_withtable(){
    breathers "stop"
    go_home_withtable_helper 2.0
    sleep 2.5
    breathers "start"
}

go_reachingstart_withtable(){
    breathers "stop"
    go_reachingstart_withtable_helper 2.0
    sleep 2.5
    breathers "start"
}

go_home() {
    breathers "stop"
    go_home_helper 2.0
    sleep 2.5
    breathers "start"
}

greet_with_right_thumb_up() {
    breathers "stop"
    echo "ctpq time 1.0 off 0 pos (-44.0 36.0 54.0 91.0 -45.0 0.0 12.0 21.0 14.0 0.0 0.0 59.0 140.0 80.0 125.0 210.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.5 && smile && sleep 1.5
    go_home_helper 1.5
    sleep 2.0
    breathers "start"
}

greet_with_left_thumb_up() {
    breathers "stop"
    echo "ctpq time 2.0 off 0 pos (-44.0 36.0 54.0 91.0 -45.0 0.0 12.0 21.0 14.0 0.0 0.0 59.0 140.0 80.0 125.0 210.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.5 && smile && sleep 1.5
    go_home_helperL 1.5
    breathersL "start"
    head "start"
}

greet_like_god() {
    breathers "stop"
    echo "ctpq time 1.5 off 0 pos (-70.0 40.0 -7.0 100.0 60.0 -20.0 2.0 20.0 29.0 3.0 11.0 3.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (-70.0 40.0 -7.0 100.0 60.0 -20.0 2.0 20.0 29.0 3.0 11.0 3.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    echo "ctpq time 0.7 off 0 pos (-70.0 50.0 -30.0 80.0 40.0 -5.0 10.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 0.7 off 0 pos (-70.0 50.0 -30.0 80.0 40.0 -5.0 10.0)" | yarp rpc /ctpservice/left_arm/rpc
    # speak "Buongiorno capo!"
    speak "Bentornato. capo!"
    echo "ctpq time 0.7 off 0 pos (-70.0 40.0 -7.0 100.0 60.0 -20.0 2.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 0.7 off 0 pos (-70.0 40.0 -7.0 100.0 60.0 -20.0 2.0)" | yarp rpc /ctpservice/left_arm/rpc

    # echo "ctpq time 0.7 off 0 pos (-70.0 50.0 -30.0 80.0 40.0 -5.0 10.0)" | yarp rpc /ctpservice/right_arm/rpc
    # echo "ctpq time 0.7 off 0 pos (-70.0 50.0 -30.0 80.0 40.0 -5.0 10.0)" | yarp rpc /ctpservice/left_arm/rpc

    # echo "ctpq time 0.7 off 0 pos (-70.0 40.0 -7.0 100.0 60.0 -20.0 2.0)" | yarp rpc /ctpservice/right_arm/rpc
    # echo "ctpq time 0.7 off 0 pos (-70.0 40.0 -7.0 100.0 60.0 -20.0 2.0)" | yarp rpc /ctpservice/left_arm/rpc
    # sleep 1.5 && smile
    sleep 1.0 && smile

    go_home_helper 2.0
}

hold_pennello() {
    echo "TODO"
}

smolla_pennello() {
    echo "TODO"
}

grasp_apple() {
    echo "ctpq time 1.5 off 0 pos (-46.0 27.0 -2.0 65.0 -80.0 -24.0 11.0 17.0 87.0 0.0 52.0 14.0 77.0 13.0 73.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
}

release_apple() {
    echo "release_apple TODO"
}

blinkWithEmotions() {
    echo "set all sur" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set all hap" | yarp rpc /icub/face/emotions/in
}
blinkDoubleWithEmotions() {
    echo "set all sur" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set all hap" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set all sur" | yarp rpc /icub/face/emotions/in
    sleep 0.1
    echo "set all hap" | yarp rpc /icub/face/emotions/in
}


mostra_muscoli() {
    breathers "stop"
    echo "ctpq time 1.5 off 0 pos (-27.0 78.0 -37.0 33.0 -79.0 0.0 -4.0 26.0 27.0 0.0 29.0 59.0 117.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (-27.0 78.0 -37.0 33.0 -79.0 0.0 -4.0 26.0 27.0 0.0 29.0 59.0 117.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 1.0 off 0 pos (-27.0 78.0 -37.0 93.0 -79.0 0.0 -4.0 26.0 67.0 0.0 99.0 59.0 117.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.0 off 0 pos (-27.0 78.0 -37.0 93.0 -79.0 0.0 -4.0 26.0 67.0 0.0 99.0 59.0 117.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
    # speak "Dei supereroi"
    sleep 3.0
    smile
    go_home_helper 2.0
    breathers "start"
}

graspa_volante() {
    # echo "ctpq time 1.5 off 0 pos (-45.0 19.0 11.0 55.0 2.0 -3.0 -17.0 12.0 53.0 0.0 91.0  61.0 106.0 71.0 114.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
    # echo "ctpq time 1.5 off 0 pos (-45.0 19.0 11.0 58.0 2.0 -5.0 -9.0  10.0 54.0 2.0 106.0 64.0 111.0 61.0 100.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc

    breathers "stop"

    echo "ctpq time 1.5 off 0 pos (0.0 -15.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 1.5 off 0 pos (0.0 -15.0 0.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 1.5 off 0 pos (0.0  15.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 1.5 off 0 pos (0.0  15.0 0.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 1.5 off 0 pos (0.0  0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
    echo "ctpq time 1.5 off 0 pos (0.0  0.0 0.0)" | yarp rpc /ctpservice/head/rpc
    sleep 2.0 && blink
    sleep 2.0 && blink
    smile
    head "start"
}

graspa_pallina() {
    breathers "stop"
    echo "ctpq time 2.0 off 0 pos (-38.0 25.0 25.0 45.0 59.0 -11.0 -20.0 30.0 28.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 0.75 off 0 pos (-30.0 0.0 -15.0 -10.0 0.0 10.0)" | yarp rpc /ctpservice/head/rpc
    sleep 2.5
    echo "ctpq time 1.5 off 0 pos (-38.0 25.0 25.0 45.0 59.0 -11.0 -20.0 30.0 90.0 0.0 70.0 60.0 80.0 60.0 80.0 215.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.5
    speak "$1"
    echo "ctpq time 1.5 off 0 pos (-38.0 48.0 7.0 71.0 -11.0 0.0  2.0 30.0 90.0 0.0 70.0 60.0 80.0 60.0 80.0 215.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos (-20.0 0.0 -25.0 -10.0 10.0 10.0)" | yarp rpc /ctpservice/head/rpc
    sleep 2.0
    echo "ctpq time 1.0 off 0 pos (0.0 0.0 0.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
}

smolla_pallina() {
    echo "ctpq time 1.5 off 0 pos (-38.0 25.0 25.0 45.0 59.0 -11.0 -20.0 30.0 90.0 0.0 70.0 60.0 80.0 60.0 80.0 215.0)" | yarp rpc /ctpservice/right_arm/rpc
    smile && sleep 1.5 && smile
    echo "ctpq time 1.5 off 0 pos (-38.0 25.0 25.0 45.0 59.0 -11.0 -20.0 30.0 28.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    go_home_helper 2.0
    smile
    breathers "start"
}

passa_e_chiudi() {
    speak "aicab passa e chiude."
    sleep 2.0 && blink
}

buongiorno_capo() {
    # speak "Buongiorno? capo!"
    speak "Bentornato. capo!"
    sleep 1.0 # && blink
    sleep 0.5 && smile
}

smile() {
    echo "set all hap" | yarp rpc /icub/face/emotions/in
}

surprised() {
    echo "set mou sur" | yarp rpc /icub/face/emotions/in
    echo "set leb sur" | yarp rpc /icub/face/emotions/in
    echo "set reb sur" | yarp rpc /icub/face/emotions/in
}

sad() {
    echo "set mou sad" | yarp rpc /icub/face/emotions/in
    echo "set leb sad" | yarp rpc /icub/face/emotions/in
    echo "set reb sad" | yarp rpc /icub/face/emotions/in
}

ciao() {
    speak "Ciao! Mi chiamo aicccab."
}

vai_nello_spazio() {
    breathers "stop"
    echo "ctpq time 1.5 off 0 pos (-42.0 36.0 -12.0 101.0 -5.0 -5.0 -4.0 17.0 57.0 87.0 140.0 0.0 0.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 2.0
    smile
    go_home
}

meteo_bot() {
    breathers "stop"
    echo "ctpq time 1.5 off 0 pos (-55.0 49.0 -4.0 77.0 73.0   0.0 15.0 21.0 40.0 30.0 91.0 5.0 35.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 1.0 off 0 pos (0.0 0.0 30.0 0.0 -10.0 10.0)" | yarp rpc /ctpservice/head/rpc
    sleep 2.0
    echo "ctpq time 0.8 off 0 pos (-70.0 47.0 -3.0 55.0 81.0 -11.0  5.0 21.0 40.0 30.0 91.0 5.0 35.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 1.0 off 0 pos (0.0 0.0 30.0 0.0 -10.0 5.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 0.8 off 0 pos (-55.0 49.0 -4.0 77.0 73.0   0.0 15.0 21.0 40.0 30.0 91.0 5.0 35.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0 && blink
    smile
    echo "ctpq time 1.0 off 0 pos (0.0 0.0 0.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
    blink
    go_home
}

saluta() {
    breathers "stop"
    echo "ctpq time 1.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0 -11.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    #sleep 2.0 && speak "Salve colleghi."
    echo "ctpq time 0.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0  25.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 0.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0 -11.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 0.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0  25.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    echo "ctpq time 0.5 off 0 pos (-60.0 44.0 -2.0 96.0 53.0 -17.0 -11.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    smile
    #go_home
    smile
}


closing_remarks() {
    meteo_bot
    speak "Da aicab e' tutto. Fascicolo $1 terminato."
    sleep 1.5 && blink
    sleep 3.0 && blink && smile
    speak "In bocca al lupo meikers"
    smile
    greet_with_right_thumb_up
    smile
}

no_testa() {
    head "stop"
    echo "ctpq time 0.5 off 0 pos (0.0 0.0  15.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 0.5 off 0 pos (0.0 0.0  -5.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 0.5 off 0 pos (0.0 0.0  15.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 0.5 off 0 pos (0.0 0.0  -5.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
    echo "ctpq time 0.5 off 0 pos (0.0 0.0   5.0 0.0 0.0 5.0)" | yarp rpc /ctpservice/head/rpc
    head "start"
    go_home
}

fonzie() {
    breathers "stop"
    echo "ctpq time 1.5 off 0 pos ( -3.0 57.0   3.0 106.0 -9.0 -8.0 -10.0 22.0 10.0 10.0 20.0 62.0 146.0 90.0 130.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
    echo "ctpq time 1.5 off 0 pos ( -3.0 57.0   3.0 106.0 -9.0 -8.0 -10.0 22.0 10.0 10.0 20.0 62.0 146.0 90.0 130.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.5
    smile
    go_home
    breathers "start"
}

attacco_grafica() {
    head "stop"
    echo "ctpq time 1.0 off 0 pos (0.0 0.0 30.0 0.0 -10.0 5.0)" | yarp rpc /ctpservice/head/rpc
    speak "$1"
    sleep 2.0
    go_homeH
}

cun() {
    echo "set reb cun" | yarp rpc /icub/face/emotions/in
    echo "set leb cun" | yarp rpc /icub/face/emotions/in
}

angry() {
    echo "set all ang" | yarp rpc /icub/face/emotions/in
}
####################################################################################################################
#                              Palm orientations                                                                   #
####################################################################################################################
palm4stick() {
    echo "PALM CUS (0.140137 0.654117 -0.743298 2.849317) " | yarp rpc /handProfiler
}

palm4pass() {
    echo "PALM CUS (0.2176 0.7718 -0.5977 3.0170) " | yarp rpc /handProfiler
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
#                              Ellipses Trajectories                                                               #
####################################################################################################################
ellipse4fra() {
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (C -0.3 -0.1 0.0) (theta  1.57 1.57 7.85) (axes 0.1 0.1) (rev) (param 0.01 0.33))) " | yarp rpc /handProfiler
}

handProfile_CVPEllipse01() {
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.1) (rev) (param 0.1))) " | yarp rpc /handProfiler
}

handProfile_CVPEllipse005() {
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.1) (rev) (param 0.05))) " | yarp rpc /handProfiler
}

handProfile_CVPEllipse02() {
    echo "GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.1) (rev) (param 0.2))) " | yarp rpc /handProfiler
}

handProfile_TTPLEllipse01() {
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.1 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTPLEllipse05() {
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.5 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTPLEllipse02() {
    echo "GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.2 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTLFront001(){
    echo "GEN TTL (((O -0.25 -0.1 0.1) (A -0.35 -0.1 0.1) (B -0.25 -0.1 0.2) (C -0.25 -0.1 0.0) (theta 0.0 1.57 3.14) (axes 0.1 0.1) (param 0.01 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTLFront002(){
    echo "GEN TTL (((O -0.25 -0.1 0.1) (A -0.35 -0.1 0.1) (B -0.25 -0.1 0.2) (C -0.25 -0.1 0.0) (theta 0.0 1.57 3.14) (axes 0.1 0.1) (param 0.02 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTLFront001_rev(){
    echo "GEN TTL (((O -0.25 -0.1 0.1) (A -0.35 -0.1 0.1) (B -0.25 -0.1 0.2) (C -0.25 -0.1 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.01 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTLFront004_emp(){
    echo "GEN TTL (((O -0.25 -0.1 0.1) (A -0.35 -0.1 0.05) (B -0.25 -0.1 0.3) (C -0.25 -0.1 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.04 0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPFront008_emp(){
    echo "GEN CVP (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.3) (C -0.25 0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.08)))" | yarp rpc /handProfiler
}

handProfile_TTLEllipse001() {
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.01 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTLEllipse002() {
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.02 0.33)))" | yarp rpc /handProfiler
}

handProfile_TTLEllipse005() {
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.05 0.33)))" | yarp rpc /handProfiler
}

######################################### right hand provide object

handRightP_TTLFront001_rev(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.2) (C -0.25 0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.01 0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPFront001_rev(){
    echo "GEN CVP (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.2) (C -0.25 0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.09)))" | yarp rpc /handProfiler
}

handRightP_TTLFront004_emp(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.35 0.2 0.1) (B -0.25 0.2 0.3) (C -0.25 0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.04 0.33)))" | yarp rpc /handProfiler
}

######################################### right hand halt

handRightP_TTLHalt001_rev(){
    echo "GEN TTL (((O -0.25 0.17 0.1) (A -0.35 0.17 0.1) (B -0.25 0.17 0.11) (C -0.1 0.17 0.1) (theta 0.1 1.57 3.14) (axes 0.1 0.01) (rev) (param 0.005 0.33)))" | yarp rpc /handProfiler
}

handRightP_TTLHalt002_emp(){
    echo "GEN TTL (((O -0.25 0.17 0.1) (A -0.35 0.17 0.1) (B -0.25 0.17 0.11) (C -0.1 0.17 0.1) (theta 0.05 1.57 3.14) (axes 0.1 0.01) (rev) (param 0.0095 0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPHalt008_emp(){
    echo "GEN CVP (((O -0.25 0.17 0.1) (A -0.35 0.17 0.1) (B -0.25 0.17 0.11) (C -0.1 0.17 0.1) (theta 0.05 1.57 3.14) (axes 0.1 0.01) (rev) (param 0.12)))" | yarp rpc /handProfiler
}

########################################### right hand pointing

handRightP_TTLPoint005_emp(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.2 0.2 0.1) (B -0.25 0.2 0.25) (C -0.25 0.2 -0.5) (theta 1.2 1.57 4.0) (axes 0.05 0.15) (param 0.013 0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPPoint008_emp(){
    echo "GEN CVP (((O -0.25 0.2 0.1) (A -0.35 0.2 -0.1) (B -0.25 0.2 0.1) (C -0.25 0.2 0.0) (theta 0.125 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

handRightP_TTLPoint001_rev(){
    echo "GEN TTL (((O -0.25 0.2 0.1) (A -0.2 0.2 0.1) (B -0.25 0.2 0.2) (C -0.25 0.2 -0.0) (theta 1.3 1.57 4.0) (axes 0.05 0.1) (param 0.008 0.33)))" | yarp rpc /handProfiler
}


############################################ tests for Placing an object on a table right (todo)

handRightP_TTLTable005_emp(){
    echo "GEN TTL (((O -0.2 0.2 0.0) (A -0.2 0.2 0.2) (B -0.25 0.2 0.10) (C -0.35 0.2 0.0) (theta 0.3 1.57 2.65) (axes 0.2 0.15) (param 0.005 0.33)))" | yarp rpc /handProfiler
}

handRightP_TTLTable002(){
    echo "GEN TTL (((O -0.15 0.2 0.1) (A -0.15 0.2 0.2) (B -0.25 0.2 0.1) (C -0.15 0.2 0.0) (theta 0.0 1.57 1.70) (axes 0.1 0.1) (param 0.007  0.33)))" | yarp rpc /handProfiler
}

handRightP_CVPTable008_const_strange(){
    echo "GEN CVP (((O -0.2 0.2 0.0) (A -0.15 0.2 0.1) (B -0.35 0.2 0.1) (C -0.35 0.2 0.0) (theta 0.785 1.57 2.65) (axes 0.1 0.2) (param 0.15)))" | yarp rpc /handProfiler
}

handRightP_CVPTable008_const(){
    echo "GEN CVP (((O -0.2 0.2 0.0) (A -0.15 0.2 0.1) (B -0.35 0.2 0.1) (C -0.35 0.2 0.0) (theta 0.785 1.57 2.65) (axes 0.1 0.2) (param 0.15)))" | yarp rpc /handProfiler
}

######################################### left hand provide object

handLeftP_TTLFront001_rev(){
    echo "GEN TTL (((O -0.25 -0.2 0.1) (A -0.35 -0.2 0.1) (B -0.25 -0.2 0.2) (C -0.25 -0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.01 0.33)))" | yarp rpc /handProfiler
}

handLeftP_CVPFront001_rev(){
    echo "GEN CVP (((O -0.25 -0.2 0.1) (A -0.35 -0.2 0.1) (B -0.25 -0.2 0.2) (C -0.25 -0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.1) (rev) (param 0.09)))" | yarp rpc /handProfiler
}

handLeftP_TTLFront004_emp(){
    echo "GEN TTL (((O -0.25 -0.2 0.1) (A -0.35 -0.2 0.1) (B -0.25 -0.2 0.3) (C -0.25 -0.2 0.0) (theta 0.707 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.012 0.33)))" | yarp rpc /handProfiler
}

newHandLeftProvide(){
    echo "GEN TTL (((O -0.3 -0.1 0.1) (A -0.37 -0.1 0.1) (B -0.3 -0.1 0.13) (C -0.23 -0.1 0.1) (theta 0.0 1.57 3.14) (axes 0.07 0.03) (rev) (param 0.01 0.33)))" | yarp rpc /handProfiler
}

######################################### left hand halt

handLeftP_TTLHalt001_rev(){
    echo "GEN TTL (((O -0.25 -0.17 0.1) (A -0.35 -0.17 0.1) (B -0.25 -0.17 0.11) (C -0.1 -0.17 0.1) (theta 0.1 1.57 3.14) (axes 0.1 0.01) (rev) (param 0.003 0.33)))" | yarp rpc /handProfiler
}

handLeftP_TTLHalt002_emp(){
    echo "GEN TTL (((O -0.25 -0.17 0.1) (A -0.35 -0.17 0.1) (B -0.25 -0.17 0.11) (C -0.1 -0.17 0.1) (theta 0.05 1.57 3.14) (axes 0.1 0.01) (rev) (param 0.0095 0.33)))" | yarp rpc /handProfiler
}

handLeftP_CVPHalt008_emp(){
    echo "GEN CVP (((O -0.25 -0.17 0.1) (A -0.35 -0.17 0.1) (B -0.25 -0.17 0.11) (C -0.1 -0.17 0.1) (theta 0.05 1.57 3.14) (axes 0.1 0.01) (rev) (param 0.12)))" | yarp rpc /handProfiler
}

########################################### left hand pointing

handLeftP_TTLPoint005_emp(){
    echo "GEN TTL (((O -0.25 -0.2 0.1) (A -0.2 -0.2 0.1) (B -0.25 -0.2 0.25) (C -0.25 -0.2 -0.5) (theta 1.2 1.57 4.0) (axes 0.05 0.15) (param 0.013 0.33)))" | yarp rpc /handProfiler
}

handLeftP_CVPPoint008_emp(){
    echo "GEN CVP (((O -0.25 -0.2 0.1) (A -0.35 -0.2 -0.1) (B -0.25 -0.2 0.1) (C -0.25 -0.2 0.0) (theta 0.125 1.57 3.14) (axes 0.1 0.2) (rev) (param 0.1)))" | yarp rpc /handProfiler
}

handLeftP_TTLPoint001_rev(){
    echo "GEN TTL (((O -0.25 -0.2 0.1) (A -0.2 -0.2 0.1) (B -0.25 -0.2 0.2) (C -0.25 -0.2 -0.0) (theta 1.3 1.57 4.0) (axes 0.05 0.1) (param 0.008 0.33)))" | yarp rpc /handProfiler
}


############################################ tests for Placing an object on a table left (todo)

handLeftP_TTLTable005_emp(){
    echo "GEN TTL (((O -0.2 0.2 0.0) (A -0.2 0.2 0.2) (B -0.25 0.2 0.10) (C -0.35 0.2 0.0) (theta 0.3 1.57 2.65) (axes 0.2 0.15) (param 0.005 0.33)))" | yarp rpc /handProfiler
}

handLeftP_TTLTable002(){
    echo "GEN TTL (((O -0.15 -0.14 0.05) (A -0.15 -0.14 0.08) (B -0.30 -0.14 0.05) (C -0.15 -0.14 0.02) (theta 0.0 1.57 1.58) (axes 0.03 0.15) (param 0.008  0.33)))" | yarp rpc /handProfiler
}

handLeftP_CVPTable008_const_strange(){
    echo "GEN CVP (((O -0.2 0.2 0.0) (A -0.15 0.2 0.1) (B -0.35 0.2 0.1) (C -0.35 0.2 0.0) (theta 0.785 1.57 2.65) (axes 0.1 0.2) (param 0.15)))" | yarp rpc /handProfiler
}

handLeftP_CVPTable008_const(){
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

    reachASequence_noHead() {
        go_home_withtable
        sleep 2.0 && speak "ragazzi.... prendo un nuovo giocattolo"
        sleep 3.0 && reachA_noHead
        go_home_withtable
    }

    reachASequence() {
        go_home_withtable
        sleep 2.0 && speak "ragazzi.... prendo un nuovo giocattolo"
        sleep 3.0 && reachA
        sleep 3.0 && go_home_withtable
    }

    reachBSequence_noHead() {
        go_home_withtable
        sleep 2.0 && speak "ragazzi.... prendo un nuovo giocattolo"
        sleep 3.0 && reachB_noHead
        go_home_withtable
    }

    reachBSequence() {
        go_home_withtable
        sleep 2.0 && speak "ragazzi.... prendo un nuovo giocattolo"
        sleep 3.0 && reachB
        sleep 3.0 && go_home_withtable
    }

    reachCSequence_noHead() {
        go_home_withtable
        sleep 2.0 && speak "ragazzi.... prendo un nuovo giocattolo"
        sleep 3.0 && reachC_noHead
        go_home_withtable
    }

    reachCSequence() {
        go_home_withtable
        sleep 2.0 && speak "ragazzi.... prendo un nuovo giocattolo"
        sleep 3.0 && reachC
        sleep 3.0 && go_home_withtable
    }

    reachDSequence_noHead() {
        go_home_withtable
        sleep 2.0 && speak "ragazzi.... prendo un nuovo giocattolo"
        sleep 3.0 && reachD_noHead
        go_home_withtable
    }

    reachDSequence() {
        go_home_withtable
        sleep 2.0 && speak "ragazzi.... prendo un nuovo giocattolo"
        sleep 3.0 && reachD
        sleep 3.0 && go_home_withtable
    }

    exercise() {
        go_home_withtable
        sleep 3.0 && waving
        sleep 1.0 && ciao
        sleep 1.0 && smile
        sleep 1.0 && introduceExercise
        sleep 1.0 && go_home_withtable
        sleep 1.0 && tiltHead
        sleep 3.0 && smile
        sleep 3.0 && explainExercise
        sleep 0.5 && reachRisma
        sleep 3.0 && go_home_withtable
        echo "[info] smile"
        smile
        sleep 3.0 && getReadyToPlay
        sleep 6.0 && smile
    }

    exercise_robotic() {
        go_home_withtable
        sleep 3.0 && waving_robotic
        sleep 1.0 && ciao
        sleep 1.0 && smile
        sleep 1.0 && introduceExercise
        sleep 1.0 && go_home_withtable
        sleep 1.0 && tiltHead_robotic
        sleep 3.0 && smile
        sleep 3.0 && explainExercise
        sleep 0.5 && reachRisma_robotic
        sleep 3.0 && go_home_withtable
        echo "[info] smile"
        smile
        sleep 3.0 && getReadyToPlay
        sleep 6.0 && smile
    }

    exercise_onlyEyes() {
        go_home_withtable
        blockNeck
        sleep 3.0 && waving
        sleep 1.0 && ciao
        sleep 1.0 && smile
        sleep 1.0 && introduceExercise
        sleep 1.0 && go_home_withtable
        sleep 3.0 && smile
        sleep 3.0 && explainExercise
        sleep 0.5 && reachRisma
        sleep 3.0 && go_home_withtable
        echo "[info] smile"
        smile
        sleep 3.0 && getReadyToPlay
        sleep 6.0 && smile
        releaseNeck
    }

    exercise_beeping() {
        go_home_withtable
        sleep 3.0 && waving
        sleep 1.0 && ciao_beeping
        sleep 1.0 && smile
        sleep 1.0 && introduceExercise_beeping
	sleep 1.0 && go_home_withtable
        sleep 1.0 && tiltHead
        sleep 3.0 && smile
        sleep 3.0 && explainExercise_beeping
        sleep 0.5 && reachRisma
        sleep 3.0 && go_home_withtable
        echo "[info] smile"
        smile
        sleep 3.0 && getReadyToPlay_beeping
        sleep 6.0 && smile
    }

    horizTTPL02() {
        handProfile_TTPLHoriz02
        sleep 2.0 && handProfile_STAREXE
    }

    horizRithmTTPL02() {
        handProfile_TTPLHoriz02
        sleep 2.0 && handProfile_STAREXE
	handProfile_TTPLHoriz02
        sleep 2.0 && handProfile_STAREXE
    }

    frontTTL001() {
        handProfile_TTPLFront001
        sleep 2.0 && handProfile_STAREXE
    }

    horizTTPL001() {
        handProfile_TTPLHoriz001
        sleep 2.0 && handProfile_STAREXE
    }

    horizTTPL08() {
        handProfile_TTPLHoriz08
        sleep 2.0 && handProfile_STAREXE
    }

    horizTTPL12() {
        handProfile_TTPLHoriz12
        sleep 2.0 && handProfile_STAREXE
    }

    horizCVP005() {
        handProfile_CVPHoriz005
        sleep 2.0 && handProfile_STAREXE
    }

    horizCVP01() {
        handProfile_CVPHoriz01
        sleep 2.0 && handProfile_STAREXE
    }

    horizCVP02() {
        handProfile_CVPHoriz02
        sleep 2.0 && handProfile_STAREXE
    }

    ellipseTTL005() {
        handProfile_TTLEllipse005
        sleep 2.0 && handProfile_STAREXE
    }

    ellipseTTL002() {
        handProfile_TTLEllipse002
        sleep 2.0 && handProfile_STAREXE
    }

    ellipseTTPL04() {
        handProfile_TTPLEllipse04
        sleep 2.0 && handProfile_STAREXE
    }

    ellipseTTPL02() {
        handProfile_TTPLEllipse02
        sleep 2.0 && handProfile_STAREXE
    }

    ellipseCVP01() {
        handProfile_CVPEllipse01
        sleep 2.0 && handProfile_STAREXE
    }



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
	echo "ctpq time 1.5 off 0 pos (9.4 27.3 -29.3 103.7 -42 24.1 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palm4stick
	handRightP_TTLFront001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    provideObject001Right_const() {
	echo "ctpq time 1.5 off 0 pos (9.4 27.3 -29.3 103.7 -42 24.1 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palm4stick
	handRightP_CVPFront001_rev
        sleep 2.0 && handProfile_STAREXE
    }

     provideObjectEmpRight() {
	echo "ctpq time 1.5 off 0 pos (9.4 27.3 -29.3 103.7 -42 24.1 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palm4stick
	handRightP_TTLFront004_emp
        sleep 2.0 && handProfile_STAREXE
    }

    haltRight04(){
	echo "ctpq time 1.5 off 0 pos (9.5 27.3 -4.2 75.5 40.0 -79.7 -14.8 26.4 26.4 0.0 28.9 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palmfaceright
	handRightP_TTLHalt001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    haltRight02Emp(){
	echo "ctpq time 1.5 off 0 pos (9.5 27.3 -4.2 75.5 40.0 -79.7 -14.8 26.4 26.4 0.0 28.9 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palmfaceright
	handRightP_TTLHalt002_emp
        sleep 2.0 && handProfile_STAREXE
    }

    haltRight08Const(){
	echo "ctpq time 1.5 off 0 pos (9.5 27.3 -4.2 75.5 40.0 -79.7 -14.8 26.4 26.4 0.0 28.9 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palmfaceright
	handRightP_CVPHalt008_emp
        sleep 2.0 && handProfile_STAREXE
    }

    pointRight04(){
	echo "ctpq time 1.5 off 0 pos (-8.4 43.4 13.6 97.3 -21.6 -8.4 24.8 26.0 27.0 0.0 29.0 0.0 0.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palm4stick
	handRightP_TTLPoint001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    pointRight05Emp(){
	echo "ctpq time 1.5 off 0 pos (-8.4 43.4 13.6 97.3 -21.6 -8.4 24.8 26.0 27.0 0.0 29.0 0.0 0.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palm4stick
	handRightP_TTLPoint005_emp
        sleep 2.0 && handProfile_STAREXE
    }

    pointRight08Const(){
	echo "ctpq time 1.5 off 0 pos (-8.4 43.4 13.6 97.3 -21.6 -8.4 24.8 26.0 27.0 0.0 29.0 0.0 0.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 2.0
	palm4stick
	handRightP_CVPPoint008_emp
        sleep 2.0 && handProfile_STAREXE
    }

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

    ################################################################ left arm

    provideObject001Left() {
	echo "ctpq time 1.5 off 0 pos (9.4 27.3 -29.3 103.7 -42 24.1 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palm4stick
	handLeftP_TTLFront001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    provideObject001Left_const() {
	echo "ctpq time 1.5 off 0 pos (9.4 27.3 -29.3 103.7 -42 24.1 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palm4stick
	handLeftP_CVPFront001_rev
        sleep 2.0 && handProfile_STAREXE
    }

     provideObjectEmpLeft() {
	echo "ctpq time 1.5 off 0 pos (9.4 27.3 -29.3 103.7 -42 24.1 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palm4stick
	handLeftP_TTLFront004_emp
        sleep 2.0 && handProfile_STAREXE
    }

    haltLeft04(){
	echo "ctpq time 1.5 off 0 pos (9.5 27.3 -4.2 75.5 40.0 -79.7 -14.8 26.4 26.4 0.0 28.9 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palmface
	handLeftP_TTLHalt001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    haltLeft02Emp(){
	echo "ctpq time 1.5 off 0 pos (9.5 27.3 -4.2 75.5 40.0 -79.7 -14.8 26.4 26.4 0.0 28.9 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palmface
	handLeftP_TTLHalt002_emp
        sleep 2.0 && handProfile_STAREXE
    }

    haltLeft08Const(){
	echo "ctpq time 1.5 off 0 pos (9.5 27.3 -4.2 75.5 40.0 -79.7 -14.8 26.4 26.4 0.0 28.9 0.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palmface
	handLeftP_CVPHalt008_emp
        sleep 2.0 && handProfile_STAREXE
    }

    pointLeft04(){
	echo "ctpq time 1.5 off 0 pos (-8.4 43.4 13.6 97.3 -21.6 -8.4 24.8 26.0 27.0 0.0 29.0 0.0 0.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palm4stick
	handLeftP_TTLPoint001_rev
        sleep 2.0 && handProfile_STAREXE
    }

    pointLeft05Emp(){
	echo "ctpq time 1.5 off 0 pos (-8.4 43.4 13.6 97.3 -21.6 -8.4 24.8 26.0 27.0 0.0 29.0 0.0 0.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palm4stick
	handLeftP_TTLPoint005_emp
        sleep 2.0 && handProfile_STAREXE
    }

    pointLeft08Const(){
	echo "ctpq time 1.5 off 0 pos (-8.4 43.4 13.6 97.3 -21.6 -8.4 24.8 26.0 27.0 0.0 29.0 0.0 0.0 87.0 176.0 250.0)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	palm4stick
	handLeftP_CVPPoint008_emp
        sleep 2.0 && handProfile_STAREXE
    }

    placeTableLeft() {
	palm4stick
	handLeftP_TTLTable002
        sleep 2.0 && handProfile_STAREXE
    }

    placeTableLeftEmp() {
	palm4stick
	handLeftP_TTLTable005_emp
        sleep 2.0 && handProfile_STAREXE
    }

    placeTableLeftConst() {
	palm4stick
	handLeftP_CVPTable008_const
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

    gobackhome(){
	breathers "stop"
    	echo "ctpq time 1.5 off 0 pos (0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
	echo "ctpq time 1.5 off 0 pos (9.4 27.3 -29.3 103.7 -42 24.1 -14.8 34.0 27.2 0.0 56.0 20.0 83.3 47.7 62.9 130.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep 3.0
	breathers "start"

    }

    homeRobotic(){
	breathers "stop"
	echo "ctpq time 1.5 off 0 pos (-29.5 30.6 0.0 44.5 0.0 0.0 0.0 15.0 44.8 6.2 15.3 5.3 11.9 9.0 17.0 5.0 )" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 1.5 off 0 pos (-29.5 30.6 0.0 44.5 0.0 0.0 0.0 15.0 44.8 6.2 15.3 5.3 11.9 9.0 17.0 5.0 )" | yarp rpc /ctpservice/left_arm/rpc
	sleep 3.0
	breathers "start"
    }

    snakeRobotic(){
	breathers "stop"
	echo "ctpq time 1.5 off 0 pos (-28.3 22.5 16.3 54.3 68.4 -9.0 -0.6 46.2 20.0 19.8 19.8 9.9 14.4 9.9 10.8 110.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 3.0
	breathers "start"
    }

    captchaRobotic(){
	breathers "stop"
	echo "ctpq time 1.5 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 40.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 1.5 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 40.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	echo "ctpq time 1.5 off 0 pos (0.0 0.0 10.0)" | yarp rpc /ctpservice/torso/rpc
	sleep 2.0
	echo "ctpq time 1.5 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 1.5 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	echo "ctpq time 0.8 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 30.0 10.8 20.0 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 0.8 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 30.0 10.8 20.0 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 1.0
	echo "ctpq time 0.8 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 0.8 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 3.0
	breathers "start"
    }

    captchaNatural(){
	breathers "stop"
	echo "set reb sur" | yarp rpc /icub/face/emotions/in
	sleep 6.0
	echo "set all hap" | yarp rpc /icub/face/emotions/in
	sleep 3.0
	echo "ctpq time 1.5 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 40.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 1.5 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 40.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	echo "ctpq time 1.5 off 0 pos (0.0 0.0 10.0)" | yarp rpc /ctpservice/torso/rpc
	sleep 2.0
	echo "ctpq time 1.5 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 1.5 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 2.0
	echo "ctpq time 0.8 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 30.0 10.8 20.0 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 0.8 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 30.0 10.8 20.0 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 1.0
	echo "ctpq time 0.8 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 0.8 off 0 pos (-28.3 11.2 0.0 49.7 75.6 0.0 0.0 60.0 44.8 6.3 15.3 9.9 10.8 9.9 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	sleep 3.0
	breathers "start"
    }

    homeNatural(){
	breathers "stop"
	echo "ctpq time 1.5 off 0 pos (-28.3 22.5 16.3 60.0 68.4 -9.0 53.0 51.0 20.0 19.8 19.8 9.9 14.4 9.9 10.8 10.8)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 1.5 off 0 pos (-28.3 22.5 16.3 60.0 68.4 -9.0 53.0 51.0 20.0 19.8 19.8 9.9 14.4 9.9 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	echo "set all hap" | yarp rpc /icub/face/emotions/in
	sleep 3.0
	breathers "start"
    }

    snakeNatural(){
	breathers "stop"
	echo "ctpq time 1.5 off 0 pos (-28.3 22.5 16.3 54.3 68.4 -9.0 -0.6 46.2 20.0 19.8 19.8 9.9 14.4 9.9 10.8 110.0)" | yarp rpc /ctpservice/right_arm/rpc
	echo "ctpq time 1.5 off 0 pos (-28.3 22.5 16.3 60.0 68.4 -9.0 53.0 51.0 20.0 19.8 19.8 9.9 14.4 9.9 10.8 10.8)" | yarp rpc /ctpservice/left_arm/rpc
	echo "ctpq time 1.5 off 0 pos (0.0 0.0 10.0)" | yarp rpc /ctpservice/torso/rpc
	sleep 2.0
	echo "ctpq time 0.8 off 0 pos (-28.3 22.5 16.3 54.3 68.4 -9.0 -0.6 46.2 20.0 19.8 19.8 18.0 42.0 9.9 10.8 110.0)" | yarp rpc /ctpservice/right_arm/rpc
	sleep 0.8
	echo "ctpq time 0.8 off 0 pos (-28.3 22.5 16.3 54.3 68.4 -9.0 -0.6 46.2 20.0 19.8 19.8 9.9 14.4 9.9 10.8 110.0)" | yarp rpc /ctpservice/right_arm/rpc
	breathers "start"
    }

    torsoAvantiIndietro(){
	breathers "stop"
	echo "ctpq time 1.5 off 0 pos (0.0 0.0 8.0)" | yarp rpc /ctpservice/torso/rpc
	sleep 3.0
	echo "ctpq time 1.5 off 0 pos (0.0 0.0 0.0)" | yarp rpc /ctpservice/torso/rpc
	breathers "start"
    }

provaWrite() {
    echo "ciao alessia" | yarp rpc /countDownInterface/inputSpeakString:i
}
#############     Repeatings     #####################################

    constant005_10() {
        sleep 2.0
        constant005
        sleep 9.0
        constant005
        sleep 9.0
        constant005
        sleep 9.0
        constant005
        sleep 9.0
        constant005
        sleep 9.0
        constant005
        sleep 9.0
        constant005
        sleep 9.0
        constant005
        sleep 9.0
        constant005
        sleep 9.0
        constant005
    }

    constant01_10() {
        sleep 2.0
        constant01
        sleep 5.0
        constant01
        sleep 5.0
        constant01
        sleep 5.0
        constant01
        sleep 5.0
        constant01
        sleep 5.0
        constant01
        sleep 5.0
        constant01
        sleep 5.0
        constant01
        sleep 5.0
        constant01
        sleep 5.0
        constant01
    }

     constant02_10() {
        sleep 2.0
        constant02
        sleep 4.0
        constant02
        sleep 4.0
        constant02
        sleep 4.0
        constant02
        sleep 4.0
        constant02
        sleep 4.0
        constant02
        sleep 4.0
        constant02
        sleep 4.0
        constant02
        sleep 4.0
        constant02
        sleep 4.0
        constant02
    }

    synchro02_10() {
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
        sleep 5.0
        synchro02
    }

    synchro04_10() {
        sleep 2.0
        synchro04
        sleep 3.0
        synchro04
        sleep 3.0
        synchro04
        sleep 3.0
        synchro04
        sleep 3.0
        synchro04
        sleep 3.0
        synchro04
        sleep 3.0
        synchro04
        sleep 3.0
        synchro04
        sleep 3.0
        synchro04
        sleep 3.0
        synchro04
    }

    synchro04_100() {
        sleep 2.0
        synchro04_10
        sleep 3.0
        synchro04_10
        sleep 3.0
        synchro04_10
        sleep 3.0
        synchro04_10
        sleep 3.0
        synchro04_10
        sleep 3.0
        synchro04_10
        sleep 3.0
        synchro04_10
        sleep 3.0
        synchro04_10
        sleep 3.0
        synchro04_10
        sleep 3.0
        synchro04_10
    }

     synchro08_10() {
        sleep 5.0
        synchro08
        sleep 2.0
        synchro08
        sleep 2.0
        synchro08
        sleep 2.0
        synchro08
        sleep 2.0
        synchro08
        sleep 2.0
        synchro08
        sleep 2.0
        synchro08
        sleep 2.0
        synchro08
        sleep 2.0
        synchro08
        sleep 2.0
        synchro04
    }

    synchro12_10() {
        sleep 5.0
        synchro12
        sleep 1.0
        synchro12
        sleep 1.0
        synchro12
        sleep 1.0
        synchro12
        sleep 1.0
        synchro12
        sleep 1.0
        synchro12
        sleep 1.0
        synchro12
        sleep 1.0
        synchro12
        sleep 1.0
        synchro12
        sleep 1.0
        synchro12
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
