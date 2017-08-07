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
- Zero Disparity Probability Image
- Difference of Gaussian

### Step One : Preprocess Inputs
 - Get the images from Yarp Input ports : Left and Right view
 - Make copy into right_original, left_original
 - Convert RGB to YUV colorSpace for right_original and left_original :  Keep working only with **Y chanel**
 - Creat cropped Image from orignals (left and right) : 
 Rect coord -> Source Point ( imageSrcWidth/2, imageSrcHeight/2); 
 Size -> foveaSize( 100,100)
 - Copy cropped images to fovea_left and fovea_right

### Step Two : Difference of Gaussian (DOG)
**Inputs: fovea_left and fovea_right in range [0, 255]** 
**Outputs : fovea_dog_left and fovea_dog_right in range [0, 255]**

 - For the  two fovead images apply two gaussian Kernels and output the absolute difference of them

### Step Three : Processing Zero Disparity Probability Map
**Inputs : fovea_dog_left and fovea_dog_right in range [0, 255]** 
**Outputs : zeroDisparyMap and invertZeroDisparityMap in range [0, 255]**

 - On the two Difference of Gaussian calcul probability of zero disparity Map :
 	- **NDT algorithm** : 
 		- For every pixels analyze its neighbors  (**NDTSIZE**)
 		- Asign a score on the difference between their values and the current pixel and output a list of  score of size **NDTSIZE**
 		- Compare the two NDT results ( fovea_dog_left, fovea_dog_right) to calcul zero disparity probability Map : Compare one by one the two list and calcul probability with the amount of match between the two list
 		
 	- **RANK algorithm** :
 		- For every pixel store its neighbors with spread **RANKX**, **RANKY** into a list
 		- Compare the two list calculated on fovea_dog_left and fovea_dog_right to calcul zero dispairty probability
 		
 	- Rescale the probability map between [0,255]
 	- Given this zero disparity probabily map penalize with a radius ( the more away of the center lesslikely the pixel it's going to be at zero disparity)
 	- From scaled zero disparity map compute the invert image
 	
 	
 	
### Step Four : Markov Random field otpimization ( Energy Minimization )
 **Inputs : probabilityZeroDisparityMap, InvertprobabilityZeroDisparityMap**
 **Outputs : image lables/class of segmentation in range [0,1]**
 
  - First the image_class is assign to zero label everywhere
  - Calcul the initial energy :
  	- For every pixel in **image_label** compute the likehood given a label and add it to the total_initial_energy 
  
  	- Then for its 1st order neighbors (ie four pixel around the central pixel) calcul **prior_intensity_smoothness** similar neighbours should have same label. Add to the total_initial_energy
  	
  - Compute permutation of possible labels ( nmaps paremeters)
  - Given a label **a** calcul the expansion  by performing energy minimization :
  	- For evry pixel calcul the likehood of this pixel being classified with label **a**
  	- For its neighbors  calcul the probility of assigning label **a**
  		
  	- Optimize the energy to get the lowest one by finding the best configuration of labels
  	- If the computed energy is lower than the previous one update labels

  - When energy stabilize or reach iteration maximum return the **image_label**
## Dependency :
- OpenCV 
- Yarp