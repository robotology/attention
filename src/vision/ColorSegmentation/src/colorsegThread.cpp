#include "darwin/colorsegThread.h"
#include "darwin/fileutil.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

using namespace Darwin::colorseg;


colorsegThread::colorsegThread(ResourceFinder & _rf, string name, int rate) : rf(_rf),
																				 threadName(name),
																				 PeriodicThread(rate)
{
}


bool colorsegThread::threadInit()
{
	// extract parameters from properties (ini-file)
	
	//height = rf.find("height").asInt16();
	//width = rf.find("width").asInt16();
	
	string modelFile = rf.find("modelFile").asString().c_str();
	int nDim;
	load_numeric_array(modelFile.c_str(),&nK,&nDim,&W);
	if ( nDim!=8 ) {
		printf("Wrong size of array in file <%s>.\n",modelFile.c_str());
		return false;
	}
	printf("Segmentation model loaded from file <%s>, with %i labels.\n",modelFile.c_str(),nK);

	// Divide some components of SVM weights by 255 (because RGB values in I are in interval [0,255] rather than [0,1]).
	for ( int k=0; k<nK; k++ ) {
		for ( int i=1; i<5; i++ ) W[8*k+i] /= 255.0;
		for ( int i=5; i<8; i++ ) W[8*k+i] /= 255.0*255.0;
	}

    colorMapLabels = rf.find("colorMap").asList();

	string colormapFile = rf.find("colormapFile").asString().c_str();
	int n3, nK_;
	load_numeric_array(colormapFile.c_str(),&nK_,&n3,&colormap);
	if ( n3!=3 || nK_<nK ) {
		printf("Error: Color map in file <%s> must have 3 columns and at least that many rows as <model>.\n",colormapFile.c_str());
		return false;
	}
	printf("Colormap loaded from file <%s>.\n",colormapFile.c_str());

	smoothness = (float)rf.find("smoothness").asFloat32();
	niter = rf.find("niter").asInt16();
	minsize = (unsigned)rf.find("minsize").asInt16();
	
	q = NULL;

	// open thread input and output ports:
	if(    !cam_in.open(("/"+threadName+"/img:i").c_str()) 
		|| !cam_out.open(("/"+threadName+"/img:o").c_str())
		|| !output.open(("/"+threadName+"/out").c_str()) )
	{
		printf("Error: unable to open input or output port\n");
		return false;
	}



	return true;
}


void colorsegThread::init_onsize(int width_, int height_)
{
	width = width_;
	height = height_;

	q = new real[nK*width*height]; // allocate unary potentials
	E = grid_graph(width,height,&nE); // construct image graph
	
	f = new real[nK*2*nE]; // allocate and initialize messages
	for ( int i=0; i<nK*2*nE; i++ ) f[i]=0;
	
	K = new unsigned char[width*height];
	J = new unsigned[width*height];
}


void colorsegThread::run()
{
    while(this->isRunning() and !this->isSuspended()){
        ImageOf<PixelRgb> *img_in = cam_in.read();
        ImageOf<PixelRgb> img_in2;
        img_in2.resize(img_in->width(), img_in->height());
        img_in2.zero();

        if ( img_in == NULL ) return;
        runSem.wait();
        if ( q == NULL ) {
//		printf("Image size known, allocating MRF arrays.\n");
            init_onsize(img_in->width(),img_in->height());
        }
        if ( (img_in->width()!=width) || (img_in->height()!=height) ) {
//		printf("Error: Image size changed during session.\n");
            return;
        }
        unsigned char *I = img_in->getRawImage();

//	printf("Constructing and optimizing MRF.\n");
        compute_unary_potentials(I,W,nK,width*height,q);
        reparameterize_unary_potentials(E,nE,q,width*height,nK,f);
        for ( int iter=0; iter<niter; iter++ ) {
            float resid = trws_potts(E,nE,smoothness,q,width*height,nK,f,.5);
//		printf("iter=%i resid=%g\n",iter,resid);
            /*float fmin=1e20, fmax=-fmin;
            for ( int i=0; i<nK*2*nE; i++ ) {
                if ( f[i]<fmin ) fmin=f[i];
                if ( f[i]>fmax ) fmax=f[i];
            }
//            printf("fmean=%g fdiff=%g\n",(fmin+fmax)/2,fmax-fmin);*/
        }
        extract_labeling(q,nK,width*height,K);

//	printf("Segmentation done. Label histogram=[");
//        int *hist = new int[nK];
//        for ( int k=0; k<nK; k++ ) hist[k]=0;
//        for ( int i=0; i<width*height; i++ ) hist[K[i]]++;
////	for ( int k=0; k<nK; k++ ) printf("%i ",hist[k]);
////	printf("]\n");
//        delete[] hist;

//	printf("Finding connected components.\n");
        for ( int i=0; i<width; i++ ) K[i] = 0; // Set column 0 to background
        for ( int j=0; j<height; j++ ) K[j*width] = 0; // Set row 0 to background
        unsigned int ncomponents = connected_components(K,width,height,minsize,J);
//	printf("No. of components (excl. background) = %i\n",ncomponents);

//	printf("Computing bounding boxes.\n");
        int *bbox = new int[5*ncomponents];
        bounding_boxes(J,K,width,height,ncomponents,bbox);
        for ( int k=0; k<ncomponents; k++ ) {
            int *b = bbox + 5*k;
            //printf("%i %i %i %i %i\n",b[0],b[1],b[2],b[3],b[4]);
            for ( int i=b[0]; i<=b[1]; i++ ) K[i+width*b[2]] = K[i+width*b[3]] = b[4];
            for ( int j=b[2]; j<=b[3]; j++ ) K[b[0]+width*j] = K[b[1]+width*j] = b[4];
        }

        unsigned char *I2 = img_in2.getRawImage();
        // Apply color map and send image to output.
        for ( int t=0; t<width*height; t++ ) {
            unsigned char Kt = K[t], *It = I2 + 3*t;
            for ( int c=0; c<3; c++ ) if ( Kt>0 ) It[c] = colormap[3*Kt+c];
        }
        cam_out.prepare() = img_in2;
        cam_out.write();


        Bottle list_color_labels;
        list_color_labels.clear();
        Bottle bot;
//        list_color_labels.addInt16(ncomponents); // number of bounding boxes
        for ( int i=0; i<ncomponents; i++ ){
            for (int j =0; j < 4; ++j){
                bot.addInt16(bbox[5*i + j]);
            }

            bot.addString(colorMapLabels->get(bbox[i * 5 + 4]).asString());
            list_color_labels.addList().read(bot);
            bot.clear();

        }
        output.write(list_color_labels);



//	printf("\n");
        runSem.post();
    }

}


void colorsegThread::threadRelease()
{

    delete[] J;
    delete[] K;
    delete[] f;
    delete[] E;
    delete[] q;
    delete[] colormap;
    delete colorMapLabels;
    delete[] W;

	// interrupt ports
	cam_in.close();
	cam_out.close();
	output.close();

}


void colorsegThread::stop()
{


    // interrupt ports
    cam_in.interrupt();
    cam_out.interrupt();
    output.interrupt();


}