#ifndef __OVIMAGET_H
#define __OVIMAGET_H

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include "OvImageAdapter.h"

///Internal image class with overloaded math operators and utility functions.
/** 
* The OvImageT class is a template class used internally by the library,
* although it can easily be used for your own purposes. Arithmetic, comparison, logical, etc.
* operators are overloaded to perform per-pixel operations using a simple syntax. Mathematical
* functions (e.g., sin, exp, pow, etc.) are also overloaded. Common image operations such as
* 2D convolution, linear and nonlinear filtering, and several standard Matlab-style functions
* have been provided. 
*
* @author Abhijit Ogale
*/
template<typename T>
class OvImageT
{
public:

  //constructors and destructors
  OvImageT(); //create empty image
  OvImageT(int height, int width, int nColorChannels);  //create image of given dimensions
  OvImageT(const OvImageT<T>& srcImage, bool CopyData=true); //create image with same size as input image, with the option of copying data
  template<typename C> OvImageT(const OvImageT<C>& srcImage, bool CopyData=true); //create image with same size as an input image of a different type (i.e., int, float, etc.), with the option of copying data
  virtual ~OvImageT();

  void getDimensions(int & height, int & width, int & nColorChannels) const; //get image size
  int getHeight() const;
  int getWidth() const;
  int getChannels() const;
  void resetDimensions(int height, int width, int nColorChannels = 1); //reset image dimensions and fill with zeros
  void reshape(int height, int width, int nColorChannels = 1);  //reshape image without changing number of pixels
  void normalizeIntensityRange(); //rescale image intensities to lie between 0 and 1


  //for debugging
  void print(void); /**< print image contents (only for debugging) */

  //special operators	
  inline T& operator() (int row, int column = 0, int channel = 0); //allows easy indexing into the image, e.g., im(i,j,k)
  inline T& operator() (int row, int column = 0, int channel = 0) const; //const version of above operator
  OvImageT<T>& operator = (const OvImageT<T> & rhsImage); //assignment operator (e.g., i1 = i2; )
  OvImageT<T>& operator = (const T & rhs); //assignment operator with scalar rhs (e.g., i1 = 5.2; )
  template <typename C> OvImageT<T>& operator = (const OvImageT<C> & rhsImage); //convert from one template type to another (e.g., float to int)

  //special copying methods
  bool copyFromAdapter(const OvImageAdapter & iadapter); //import values from OvImageAdapter
  bool copyToAdapter(OvImageAdapter & iadapter); //export values to OvImageAdapter if it has same dimensions
  bool copyMasked(const OvImageT<bool> & mask, const OvImageT<T> & srcImage); //copy values from srcImage only for pixels where mask is set to true
  bool copyMasked(const OvImageT<bool> & mask, const T & value); //set pixels = value only where mask is set to true
  bool copyChannel(OvImageT<T> & input, int inputchannel, int outputchannel); //copies a certain input channel to a certain output channel
  //bool copyRegion(const T & value, int rowLo=-1, int rowHi=-1, int columnLo=-1, int columnHi=-1, int channelLo=-1, int channelHi=-1);
  //bool copyRegionEx(const T & value, int rowLo=-1, int rowHi=-1, int columnLo=-1, int columnHi=-1, int channelLo=-1, int channelHi=-1);	

  const OvImageT<T> getSubImage(int rowLo=-1, int rowHi=-1, int columnLo=-1, int columnHi=-1, int channelLo=-1, int channelHi=-1); //copy and return rectangular image block

  //arithmetic operators 
  OvImageT<T> & operator += (const OvImageT<T> & rhs);	/**< e.g., i1 += i2;*/
  OvImageT<T> & operator += (const T & rhs);				/**< e.g., i1 += 3.2;*/
  OvImageT<T> & operator -= (const OvImageT<T> & rhs);	/**< e.g., i1 -= i2;*/
  OvImageT<T> & operator -= (const T & rhs);				/**< e.g., i1 -= 5.5;*/
  OvImageT<T> & operator *= (const OvImageT<T> & rhs);	/**< e.g., i1 *= i2;*/
  OvImageT<T> & operator *= (const T & rhs);				/**< e.g., i1 *= 2;*/
  OvImageT<T> & operator /= (const OvImageT<T> & rhs);	/**< e.g., i1 /= i2;*/
  OvImageT<T> & operator /= (const T & rhs);				/**< e.g., i1 /= 10;*/

  OvImageT<T> & operator ++ (); 							/**< e.g., ++i1;*/
  OvImageT<T> & operator -- ();							/**< e.g., --i1;*/
  const OvImageT<T> operator ++ (int); 					/**< e.g., i1++;*/
  const OvImageT<T> operator -- (int);					/**< e.g., i1--;*/

  const OvImageT<T> operator - (); 						/**< e.g., i1 = -i2;*/
  const OvImageT<T> operator + ();						/**< e.g., i1 = +i2;*/

  template<typename C> friend const OvImageT<C> operator + (const OvImageT<C> & i1, const OvImageT<C> & i2);  /**< e.g., i1 = i2+i3; */
  template<typename C> friend const OvImageT<C> operator + (const double i1, const OvImageT<C> & i2);			/**< e.g., i1 = 5.2+i3; */
  template<typename C> friend const OvImageT<C> operator + (const OvImageT<C> & i1, const double i2);			/**< e.g., i1 = i2+5.2; */
  template<typename C> friend const OvImageT<C> operator - (const OvImageT<C> & i1, const OvImageT<C> & i2);	/**< e.g., i1 = i2-i3; */
  template<typename C> friend const OvImageT<C> operator - (const double i1, const OvImageT<C> & i2);			/**< e.g., i1 = 5.2-i3; */
  template<typename C> friend const OvImageT<C> operator - (const OvImageT<C> & i1, const double i2);			/**< e.g., i1 = i2-5.2; */
  template<typename C> friend const OvImageT<C> operator * (const OvImageT<C> & i1, const OvImageT<C> & i2);	/**< e.g., i1 = i2*i3; */
  template<typename C> friend const OvImageT<C> operator * (const double i1, const OvImageT<C> & i2);			/**< e.g., i1 = 10*i3; */
  template<typename C> friend const OvImageT<C> operator * (const OvImageT<C> & i1, const double i2);			/**< e.g., i1 = i2*10; */
  template<typename C> friend const OvImageT<C> operator / (const OvImageT<C> & i1, const OvImageT<C> & i2);  /**< e.g., i1 = i2/i3; */
  template<typename C> friend const OvImageT<C> operator / (const double i1, const OvImageT<C> & i2);			/**< e.g., i1 = 1/i3; */
  template<typename C> friend const OvImageT<C> operator / (const OvImageT<C> & i1, const double i2);			/**< e.g., i1 = i2/5; */

  //logical operators
  template<typename C> friend const OvImageT<bool> operator < (const OvImageT<C> & i1, const OvImageT<C> & i2);	/**< e.g., iresult = i1<i2; */
  template<typename C> friend const OvImageT<bool> operator < (const double i1, const OvImageT<C> & i2);			/**< e.g., iresult = 2<i2; */
  template<typename C> friend const OvImageT<bool> operator < (const OvImageT<C> & i1, const double i2);			/**< e.g., iresult = i1<2; */
  template<typename C> friend const OvImageT<bool> operator <= (const OvImageT<C> & i1, const OvImageT<C> & i2);	/**< e.g., iresult = i1<=i2; */
  template<typename C> friend const OvImageT<bool> operator <= (const double i1, const OvImageT<C> & i2);			/**< e.g., iresult = 2<=i2; */
  template<typename C> friend const OvImageT<bool> operator <= (const OvImageT<C> & i1, const double i2);			/**< e.g., iresult = i1<=2; */
  template<typename C> friend const OvImageT<bool> operator > (const OvImageT<C> & i1, const OvImageT<C> & i2);	/**< e.g., iresult = i1>i2; */
  template<typename C> friend const OvImageT<bool> operator > (const double i1, const OvImageT<C> & i2);			/**< e.g., iresult = 2>i2; */
  template<typename C> friend const OvImageT<bool> operator > (const OvImageT<C> & i1, const double i2);			/**< e.g., iresult = i1>2; */
  template<typename C> friend const OvImageT<bool> operator >= (const OvImageT<C> & i1, const OvImageT<C> & i2);	/**< e.g., iresult = i1>=i2; */
  template<typename C> friend const OvImageT<bool> operator >= (const double i1, const OvImageT<C> & i2);			/**< e.g., iresult = 2>=i2; */
  template<typename C> friend const OvImageT<bool> operator >= (const OvImageT<C> & i1, const double i2);			/**< e.g., iresult = i1>=2; */
  template<typename C> friend const OvImageT<bool> operator == (const OvImageT<C> & i1, const OvImageT<C> & i2);	/**< e.g., iresult = i1==i2; */
  template<typename C> friend const OvImageT<bool> operator == (const double i1, const OvImageT<C> & i2);			/**< e.g., iresult = 2==i2; */
  template<typename C> friend const OvImageT<bool> operator == (const OvImageT<C> & i1, const double i2);			/**< e.g., iresult = i1==2; */

  //operations defined only on boolean images
  const OvImageT<bool> operator ! () const;/**< e.g., iflag1 = !iflag2;*/
  friend const OvImageT<bool> operator && (const OvImageT<bool> & i1, const OvImageT<bool> & i2); /**< e.g., iflag1 = iflag2 && iflag3; */
  friend const OvImageT<bool> operator || (const OvImageT<bool> & i1, const OvImageT<bool> & i2); /**< e.g., iflag1 = iflag2 || iflag3; */

  //math functions
  template<typename C> friend const OvImageT<C> cos (const OvImageT<C> & i1);	/**< e.g., i2 = cos(i1); */
  template<typename C> friend const OvImageT<C> sin (const OvImageT<C> & i1);	/**< e.g., i2 = sin(i1); */
  template<typename C> friend const OvImageT<C> tan (const OvImageT<C> & i1);	/**< e.g., i2 = tan(i1); */
  template<typename C> friend const OvImageT<C> acos (const OvImageT<C> & i1);	/**< e.g., i2 = acos(i1); */
  template<typename C> friend const OvImageT<C> asin (const OvImageT<C> & i1);	/**< e.g., i2 = asin(i1); */
  template<typename C> friend const OvImageT<C> atan (const OvImageT<C> & i1);	/**< e.g., i2 = atan(i1); */
  template<typename C> friend const OvImageT<C> atan2 (const OvImageT<C> & iy, const OvImageT<C> & ix); /**< e.g., i2 = atan2(iy,ix); */
  template<typename C> friend const OvImageT<C> cosh (const OvImageT<C> & i1);	/**< e.g., i2 = cosh(i1); */
  template<typename C> friend const OvImageT<C> sinh (const OvImageT<C> & i1);	/**< e.g., i2 = sinh(i1); */
  template<typename C> friend const OvImageT<C> tanh (const OvImageT<C> & i1);	/**< e.g., i2 = tang(i1); */
  template<typename C> friend const OvImageT<C> exp (const OvImageT<C> & i1);		/**< e.g., i2 = exp(i1); */
  template<typename C> friend const OvImageT<C> log (const OvImageT<C> & i1);		/**< e.g., i2 = log(i1); */
  template<typename C> friend const OvImageT<C> log10 (const OvImageT<C> & i1);	/**< e.g., i2 = log10(i1); */
  template<typename C> friend const OvImageT<C> abs (const OvImageT<C> & i1);		/**< e.g., i2 = abs(i1); */
  template<typename C> friend const OvImageT<C> ceil (const OvImageT<C> & i1);	/**< e.g., i2 = ceil(i1); */
  template<typename C> friend const OvImageT<C> floor (const OvImageT<C> & i1);	/**< e.g., i2 = floor(i1); */
  template<typename C> friend const OvImageT<C> round (const OvImageT<C> & i1);	/**< e.g., i2 = round(i1); */
  template<typename C> friend const OvImageT<C> mod (const OvImageT<C> & i1, double d);	/**< e.g., i2 = mod(i1,5); */
  template<typename C> friend const OvImageT<C> pow (const OvImageT<C> & i1, double p);	/**< e.g., i2 = pow(i1,2); */
  template<typename C> friend const OvImageT<C> pow (double p, const OvImageT<C> & i1);	/**< e.g., i2 = pow(3,i1); */
  template<typename C> friend const OvImageT<C> sqrt (const OvImageT<C> & i1);			/**< e.g., i2 = sqrt(i1); */

  //filtering, convolution, and other utility functions
  template<typename C> friend const OvImageT<C> convolve2D (const OvImageT<C> & ikernel, const OvImageT<C> & input);	//2D convolution
  template<typename C> friend const OvImageT<C> filter2D (const OvImageT<C> & ikernel, const OvImageT<C> & input);	//2D filtering
  //need to implement separable versions for greater speed with separable filters

  template<typename C> friend const OvImageT<C> medianFilter2D (const OvImageT<C> & input, int filterHeight, int filterWidth); //median filter
  template<typename C> friend const OvImageT<C> minFilter2D (const OvImageT<C> & input, int filterHeight, int filterWidth);	 //minimum filter
  template<typename C> friend const OvImageT<C> maxFilter2D (const OvImageT<C> & input, int filterHeight, int filterWidth);	 //maximum filter
  template<typename C> friend const OvImageT<C> meanFilter2D (const OvImageT<C> & input, int filterHeight, int filterWidth);	 //mean filter

  template<typename C> friend const OvImageT<C> min(const OvImageT<C> & input, int dimension);	//min along a certain dimension (1,2,3 = height, width, or color respectively), e.g., i1 = min(i2,3); returns image with same height and width but 1 color channel
  template<typename C> friend const OvImageT<C> max(const OvImageT<C> & input, int dimension);	//max along a certain dimension (1,2,3 = height, width, or color respectively), e.g., i1 = max(i2,3); returns image with same height and width but 1 color channel
  template<typename C> friend const OvImageT<C> mean(const OvImageT<C> & input, int dimension);	//mean along a certain dimension (1,2,3 = height, width, or color respectively), e.g., i1 = mean(i2,3); returns image with same height and width but 1 color channel
  template<typename C> friend const OvImageT<C> sum(const OvImageT<C> & input, int dimension);	//sum along a certain dimension (1,2,3 = height, width, or color respectively), e.g., i1 = sum(i2,3); returns image with same height and width but 1 color channel
  template<typename C> friend C sumRegion(const OvImageT<C> & input, int rowLo, int rowHi, int columnLo, int columnHi, int channelLo, int channelHi); //sum pixels in a rectangular image region
  template<typename C> friend C sumSingleChannel(const OvImageT<C> & input, int channel); //sum all pixels in a single color channel
  template<typename C> friend C sumAll(const OvImageT<C> & input); //sum all image pixels
  template<typename C> friend C L1Norm(const OvImageT<C> & input); //sum of absolute values of all pixels
  template<typename C> friend C L2Norm(const OvImageT<C> & input); //sqrt of sum of squared pixel values

  template<typename C> friend const OvImageT<C> transpose(const OvImageT<C> & input); //transpose image (each color channel independently)
  template<typename C> friend const OvImageT<C> flipLR(const OvImageT<C> & input); //flip image left to right (i.e., about vertical axis)
  template<typename C> friend const OvImageT<C> flipUD(const OvImageT<C> & input); //flip image upside-down (i.e., about horizontal axis)

  template<typename C> friend const OvImageT<C> repmat (const OvImageT<C> & input, int height, int width, int channels); //tile input image 'height' times vertically, 'width' times horizontally, and 'channels' times along color channels
  template<typename C> friend const OvImageT<C> shiftImageXY (const OvImageT<C> & input, int columns, int rows); //return copy of input image translated by (rows, columns)

  template<typename C> friend const OvImageT<C> resizeNearestNbr(const OvImageT<C> & input, double scale, bool preSmooth);	//rescale image using nearest neighbor method; use preSmooth to enable resampling
  template<typename C> friend const OvImageT<C> resizeBilinear(const OvImageT<C> & input, double scale, bool preSmooth);		//rescale image using bilinear interpolation method; use preSmooth to enable resampling

  //methods to create specific images and kernels	and their standalone friend versions
  void setToRandom(double lowerbound, double upperbound); //fill caller with random numbers
  friend const OvImageT<double> random(double lowerbound, double upperbound, int height, int width, int nColorChannels); //create new image filled with random numbers

  void setToMeshgridX (T x1, T x2, T y1, T y2, T dx = 1, T dy = 1); //set caller to an image of height y2-y1+1 and width x2-x1+1 with each pixel set to its x-coordinate (from x1 to x2)
  friend const OvImageT<double> meshgridX (double x1, double x2, double y1, double y2, double dx, double dy); //create new image of height y2-y1+1 and width x2-x1+1 with each pixel set to its x-coordinate (from x1 to x2)

  void setToMeshgridY (T x1, T x2, T y1, T y2, T dx = 1, T dy = 1); //set caller to an image of height y2-y1+1 and width x2-x1+1 with each pixel set to its y-coordinate (from y1 to y2)
  friend const OvImageT<double> meshgridY (double x1, double x2, double y1, double y2, double dx, double dy); //create new image of height y2-y1+1 and width x2-x1+1 with each pixel set to its y-coordinate (from y1 to y2)

  void setToGaussian(int size, double sigma);	 //set caller to a gaussian
  friend const OvImageT<double> gaussian(int size, double sigma);	 //create a new gaussian

  void setToGaborX(int size, double sigma, double period, double phaseshift);	//set caller to a gabor filter oriented horizontally
  friend const OvImageT<double> gaborX(int size, double sigma, double period, double phaseshift);	//create a gabor filter oriented horizontally

  void setToGaborY(int size, double sigma, double period, double phaseshift);	//set caller to a gabor filter oriented vertically
  friend const OvImageT<double> gaborY(int size, double sigma, double period, double phaseshift);	//create a gabor filter oriented vertically

  void setToGaborOriented(int size, double sigma, double period, double angle, double phaseshift); //set caller to a gabor filter with a user-specified orientation
  friend const OvImageT<double> gaborOriented(int size, double sigma, double period, double angle, double phaseshift);	//create a gabor filter with a user-specified orientation

  OvImageT<double> getGaborPhaseStack();

  void setToGray();	//convert caller to gray image (single channel)
  template<typename C> friend const OvImageT<C> rgb2gray(const OvImageT<C> & input);	 //convert color image (multiple channels) to gray image (single channel)

  template<typename C> friend bool haveEqualDimensions (const OvImageT<C> & i1, const OvImageT<C> & i2); // Returns true if the two input images have the same height, width and channels
  template<typename C> friend bool haveEqualHeightWidth (const OvImageT<C> & i1, const OvImageT<C> & i2); // Returns true if the two input images have the same height and width, ignores number of channels

  //desired functionality:
  //imtransform (for general transformation matrix)
  //generalized nonlinear block filter
  //separable filtering


protected:
  int  mHeight;			/**<height of the image*/
  int  mWidth;			/**<width of the image*/
  int  mChannels;			/**<number of color channels or dimensions (e.g., 1 for grayscale, 3 for RGB)*/

  //for convenience
  int  mHeightTimesWidth;	/**< mWidth*mHeight (for convenience) */
  int  mSize;				/**< mWidth*mHeight*mChannels (for convenience) */

  T   *mData;				/**< Image data */
};

/**  Rounds to nearest integer
* @param value	input value
* @return the rounded value
*/
inline int ov_round(double value)
{
  return int(value + 0.5);
}


#endif //__OVIMAGET_H

