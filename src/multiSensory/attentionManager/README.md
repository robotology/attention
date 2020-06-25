
# Attention Manager (attentionManager)  
  
This Module: 
*  Controls the attention process 
* Publish the details about the most selient point in the combined map which exceded a certain threshold value defined in the ini file 
  
  
### Input Ports  
---  
The module has one input ports. their names has a prefix with the module name. The defaultValue of the module name is ```/attentionManager``` The ports are as follows:   
  
| Port                                 | Port name                              | Port Type         
| -----------------------------------  |:------------------------------------:  | :----------------:|  
| Combined attention saliency map      |```/<module name>/combinedImage:i```              | yarp::sig::ImageOf< yarp::sig::PixelMono > |  
 
  
* The combined saliency map from selectiveAttentionEngine port /combination:o 
  

  
### output Ports  
---  
The module has one input ports. their names has a prefix with the module name. The defaultValue of the module name is ```/attentionManager``` The ports are as follows:   
  
| Port                                 | Port name                              | Port Type         
| -----------------------------------  |:------------------------------------:  | :----------------:|  
| the salient point which exceded the threshold      |```/<module name>/hotPoint:o```              | Bottle |  
 
  
* The output form is a list which has the x and y values and then the integer value of this pixel. example: ```(100  30) 130 ```
  

###  rpc client ports  ---  
The module has 1  rpc client ports as follows:   
  
| Port                                                    | Port name                             | 
| ------------------------------------------------------  |:-----------------------------------:  | 
| selectinve attention engine controller             |```/<module name>/engineControl:oi```             |


  
## Module running Configuration The configuration defines the running parameters of the module as follow:   
  
### Running Parameters: ---  
You can specify the context and the config file using the following command. By default, the context is logpolarAttention and the name of the config file is attentionManager.ini  
```bash  
attentionManager --context <parameter> --from <parameter>.ini  
```  
  
  
You can specify the name of the module using this command  
```bash  
attentionManager --name /<module name>  
```  
#### Config file  
  
---  
In the configuration file you need to specify 4 values in a group with name: processingParam   
  
| Parameter         | description          |  Type  |  
| -----------------------------------  |:------------------------------------  | :----------------:|  
| ```threshold```    |the considerable value in which the max point will be considered as a hot point | Int |  

  
#### config file example  
  
```ini  
[processingParam]
threshold         180
```  
  

#### Copy rights  
---  
  
 * Copyright (C)2020  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia  
 * Author: Omar Eldardeer  
 * email: omar.eldardeer@iit.it  
 * Permission is granted to copy, distribute, and/or modify this program  
 * under the terms of the GNU General Public License, version 2 or any  
 * later version published by the Free Software Foundation.  
   * A copy of the license can be found at  
   *  http://www.robotcub.org/icub/license/gpl.txt  
   * This program is distributed in the hope that it will be useful, but  
 * WITHOUT ANY WARRANTY; without even the implied warranty of  
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General  
 * Public License for more details  
  
  
#### Bug issues reporting  
---  
* Create a [new issue](https://github.com/robotology/attention/issues/new)   
* Email the author ```omar.eldardeer@iit.it```