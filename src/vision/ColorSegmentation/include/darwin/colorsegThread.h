#ifndef COLORSEG_THREAD_H
#define COLORSEG_THREAD_H

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>

#include "darwin/segment.h"

// OpenCV library:
// #include <cv.h> 

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

namespace Darwin{
namespace colorseg{

// friend class of PMP class
class colorsegThread : public PeriodicThread
{
public:
	colorsegThread(ResourceFinder &_rf, string name, int rate);
	bool threadInit();
	void run();
	void threadRelease();
	void stop();
	
private:
	BufferedPort< ImageOf<PixelRgb> > cam_in;
	BufferedPort< ImageOf<PixelRgb> > cam_out;
	Port output;

	Semaphore runSem;
	ResourceFinder rf;
	string threadName;

	Bottle *colorMapLabels;
	int width, height; // image dimensions
	int nK; // number of labels
	string colormapFile, svmFile;
	unsigned char *colormap; // maps labels to RGB-triplets
	float *W; // segmentation model (SVM weights)
	float smoothness; // smoothness prior
	int niter; // number of iterations of MRF optimizer (TRW-S)
	unsigned minsize; // minimmum size of connected components
	int nE; // number of edges in the image graph
	unsigned *E; // edges of the image graph
	real *q; // MRF unary potentials
	real *f; // TRW-S messages
	unsigned char *K; // image with labels (output of MRF optimizer)
	unsigned *J; // image with labels (output of connected components algorithm)

	void init_onsize(int width_, int height_);
};

}// end namespace colorseg
}// end namespace Darwin

#endif
