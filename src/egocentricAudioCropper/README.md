# Egocentric Audio Cropper (egocentricAudioCropper)

This module is cutting the 360° bayesian alocentric angles probability of the audio to egocentric angle probability as well as a cartesian map which defines the angle with the maximum egocentric probability.  


### Input Ports
---
The module has two input ports. their names has a prefix with the module name. The defaultValue of the module name is ```/egocentricAudioCropper``` 
The ports are as follows: 

| Port                                 | Port name                              | Port Type       
| -----------------------------------  |:------------------------------------:  | :----------------:|
| 360° bayesian angles probability      |```/<module name>/map:i```              | yarp::sig::Matrix |
| Robot's head angles                  | ```/<module name>/gazeAngles:i```      | yarp::os::Bottle  |

* 360° bayesian angles probability : this port receives a matrix object with size 1*360 which represent the angles from -180 to 180 around the center of the robot. each value represents the probability of the. This matrix can be recieved from the module [audioPreprocessor](https://github.com/TataLab/iCubAudioAttention/blob/master/modules/audioPreprocessor/doc/README.md) which is part of [Tatalab/iCubAudioAttention ](https://github.com/TataLab/iCubAudioAttention) repository
* Robot's head angles: this port should receive the angles of the head of the robot as a bottle. We are only concerning about azimuth angle of the eyes. The index of this angle should be specified in the configuration file as a parameters section. This angles can be recieved from the module [iKenGazeCtrl](http://www.icub.org/software_documentation/group__iKinGazeCtrl.html) which is part of [robotology/icub-main  ](https://github.com/robotology/icub-main)  repository this value is used to transfer the allocentric angle probability to egocentric perspective with a limited field equal to the current vision field of the robot.








### Outpot Ports 
---
The module has 2 output ports as follows: 

| Port                                                    | Port name                                  | Port Type  |
| ------------------------------------------------------  |:-----------------------------------:  | :----------------:|
| Egocentric bayesian angles probability                   |```/<module name>/map:o```             | yarp::sig::Matrix |
| Cart. egocentric audio map   | ```/<module name>/cartImg:o```         | yarp::sig::ImageOf< yarp::sig::PixelMono >  |

* Egocentric bayesian angles probability  is a sub matrix from the alocentric audio map (the main input of the module) which is based on the gaze angle and the angle of view. 
* Cart. egocentric audio map: is an Mono color image with size (1, AOV). AOV: angle of view of the camera used in the robot. The values of the image are all black except one value with white color which represent the highest angle probability of the audio.

## Module running Configuration  
The configuration defines the running parameters of the module as follow: 

### Running Parameters: 
---
You can specify the context and the config file using the following command. By default, the context is egocentricAudioCropper and the name of the config file is egocentricAudioCropper.ini
```bash
egocentricAudioCropper --context <parameter> --from <parameter>.ini
```


You can specify the name of the module using this command
```bash
egocentricAudioCropper --name /<module name>
```
#### Config file

---
In the configuration file you need to specify 4 values in a group with name: cameraParams 

| Parameter         | description          |  Type  |
| -----------------------------------  |:------------------------------------  | :----------------:|
| ```fileName```    |the name of the config file of the camera parameters (*.ini)  | string |
| ```side```        |the camera used in the vision modules (left) or (right) | string |
| ```context```     |the name of the context of the camera parameters  | string |
| ```azimuthIndex```|the index of the azimuth angle in the bottle | int |

#### config file example

```ini
[cameraParams]
fileName         icubEyes.ini
side             left
context          logpolarAttention
azimuthIndex     0
```

#### Camera parameter file config example
```ini
[CAMERA_CALIBRATION_RIGHT]

w  320
h  240
fx 220.058
fy 219.391
cx 151.955
cy 115.262
k1 -0.378088
k2 0.146602
p1 0.000361946
p2 -0.000148989

[CAMERA_CALIBRATION_LEFT]

w  320
h  240
fx 217.537
fy 216.843
cx 180.135
cy 115.726
k1 -0.372612
k2 0.145595
p1 0.000378242
p2 -0.000712398


[STEREO_DISPARITY]
HN (0.997258 0.0291087 0.0680328 -0.0636521 -0.0277951 0.99941 -0.0201755 -0.000270746 -0.0685799 0.0182293 0.997479 0.0101356 0 0 0 1)
QL ( 0.000000	 0.000000	 0.000000	-0.000220	-0.000292	 0.000068	-0.000003	 0.006898)
QR ( 0.000000	 0.000000	 0.000000	-0.000220	-0.000292	 0.000068	-0.000003	-0.008767)
```

#### Copy rights
---

  * Copyright (C)2020  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Omar Eldardeer
  * email: omar.eldardeer@iit.it
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


#### Bug issues reporting
---
* Create a [new issue](https://github.com/robotology/attention/issues/new) 
* Email the author ```omar.eldardeer@iit.it```
