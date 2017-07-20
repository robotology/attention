# Markov Random Field Zero Disparity Filter  (MRF ZDF)

## Description :
 This project implements a segmentation process using Markov random field optimization on a zero disparity area. The motivation behind this approach is to be closer of the Human vision system. 
 
 
## Algorithm
**Inputs** : 
- Left Image of the scene (RGB)
- Right Image of the scene (RGB)

**Principal Output** : 
- Segmentation Image of the scene

**Optional Outputs**:
- Probability Image
- Difference of Gaussian

### Step One : Preprocess Inputs
 - Get the images from Yarp Input ports : Left and Right view
 - Make copy into right_original, left_original
 - Convert RGB to YUV colorSpace for right_original and left_original :  Keep working only with **Y chanel**
 - Creat cropped Image from orignals (left and right) : 
 Rect coord -> Source Point ( imageSrcWidth/2, imageSrcHeight/2); 
 Size -> foveaSize( 100,100)
 - Copy cropped images to fovea_left and fovea_right

### Step Two : Difference of Gaussian
- On the two fovead images apply two gaussian Kernels.

### Step Three : 

## Dependency :
- OpenCV 
- Yarp