// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2003 Vladimir Kolmogorov (vnk@cs.cornell.edu), 2009, Andrew Dankers and Vadim Tikhanoff
*
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * GNU General Public License can be found at http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 *
*/


#include <stdio.h>
#include <time.h>
#include "iCub/multiclass.h"
#include "iCub/energy.h"

#define VAR_ACTIVE ((Energy::Var)0)

#define ALPHA_SINK
/*
  if ALPHA_SINK is defined then interpretation of a cut is as follows:
  SOURCE means initial label
  SINK   means new label \alpha

  if ALPHA_SINK is not defined then SOURCE and SINK are swapped
*/
#ifdef ALPHA_SINK
#define ADD_TERM1(var, E0, E1) add_term1(var, E0, E1)
#define ADD_TERM2(var1, var2, E00, E01, E10, E11) add_term2(var1, var2, E00, E01, E10, E11)
#define VALUE0 0
#define VALUE1 1
#else
#define ADD_TERM1(var, E0, E1) add_term1(var, E1, E0)
#define ADD_TERM2(var1, var2, E00, E01, E10, E11) add_term2(var1, var2, E11, E10, E01, E00)
#define VALUE0 1
#define VALUE1 0
#endif

void error_function(char *msg)
{
  fprintf(stderr, "%s\n", msg);
  exit(1);
}

MultiClass::MultiClass(defSize im_size_, int psb_in_, int n_, Parameters *_params)
{

  params = _params;   
  nmaps  =  n_;
  im_size.width  = im_size_.width;
  im_size.height = im_size_.height;
  psb_in = psb_in_;
  
  im_sz.x = im_size.width;
  im_sz.y = im_size.height;

  //out = ippiMalloc_8u_C1(im_size.width,im_size.height,&psb); 
  outImage = cvCreateImage(cvSize(im_size.width, im_size.height), IPL_DEPTH_8U, 1);
  out = (unsigned char *)outImage->imageData;
  psb = outImage->widthStep;

  ptr_im = (void**) malloc(im_size.width*im_size.height*sizeof(void*));
  len_nv = im_size.width;

  clear();

}

MultiClass::~MultiClass()
{
  //ippFree (out);
  free (ptr_im);
}


void MultiClass::clear()
{
  // set all the pixels to zero
  //ippiSet_8u_C1R(0,out,psb,im_size);
  
  int widthStep = outImage->widthStep;
  int height    = outImage->height;
  int width     = outImage->width;

  for( int y=0; y < height; y++ ) { 
        unsigned char* ptr = (unsigned char*) ( outImage->imageData + y * widthStep );
        for( int x=0; x < width; x++ ) { 
            ptr[x] = 0; //Set red to max (BGR format)
        }
    }
}

void MultiClass::generate_permutation(int *buf, int n)
{
    int i, j;
    
    for (i=0; i<n; i++) {
        buf[i] = i;
    }
    
    for (i=0; i<n-1; i++) {
        j = i + (int) (((double)rand()/RAND_MAX)*(n - i)); // Possible index overflow to check
        int tmp = buf[i]; buf[i] = buf[j]; buf[j] = tmp;
    }
}

void MultiClass::proc( unsigned  char* im_, unsigned char** prob_)
{
    printf("multiclass processing \n");

    im   =   im_;
    prob = prob_;


    int a;
    int buf_num;
    int label;
    int E_old;
    printf("creating random generator.... \n");
    srand(1);
    int *permutation = new  int[nmaps];
    bool *buf        = new bool[nmaps];
	int energyOut;
   
    //initial energy:
    printf("computing initial energy....... \n");
	energyOut = compute_energy();
	printf("success, energy is %d \n", energyOut);
    
    //optimise:
    printf("optimization......\n");
    for (int i=0; i<nmaps; i++) {
        buf[i] = false;
    }
    
    buf_num = nmaps;
    
    for (int iter=0; iter < params->iter_max && buf_num > 0; iter++) {
        if (iter==0 || params->randomize_every_iteration) {
            generate_permutation(permutation, nmaps);
        }
        
		printf("NMMAPS is: %d \n", nmaps);
        for (int index=0; index<nmaps; index++) {
            label = permutation[index];
            if (buf[label]) 
                continue;
            
            a = label;
			printf("label a is: %d \n", a);
            E_old = E;
            expand(a);
            
            if (E_old == E) {
                if (!buf[label]) { 
                    buf[label] = true; 
                    buf_num--;
                }
            }
            else {
                for (int i=0; i<nmaps; i++) {
                    buf[i] = false;
                }
                buf[label] = true;
                buf_num = nmaps - 1;
            }
        }
    }
    
    printf("returning from the multiclass process \n");
    delete [] permutation;
    delete [] buf;

}

int MultiClass::likelihood(Coord c, int d)
{
    //the probability of the images being the way they are,
    //given the hypothesized configuration.
    //return low penalty if likely.
    
    //prob[x][y] is in range{0..255}.
    int penalty = (int)( params->data_penalty * (1.0 - (prob[d][c.y * psb_in + c.x]/255.0)) );


//	int param_penalty = params->data_penalty;

//	double op = (double)prob[d][c.y * psb_in + c.x];
//
//	op = op / 255.0;
//
//	int penalty = (int)(param_penalty * (1.0 - op));
	//printf("%d\n", penalty);


    return penalty;
    
}

int MultiClass::prior_intensity_smoothness(Coord p, Coord np, int label, int nlabel) {

  //configuration preferences
  //intensity smoothness: similar neighbours should have the same label.
  //NEIGHBOURHOOD SYSTEM
  
  int penalty;

  //if hypothesized as the same, no penalty
  if (label == nlabel) {
    penalty = 0;
  }


  //Otherwise, if hypothesized different:
  else{
    
    //Always prefer continuous solutions,
    //so immediately penalise hypothetical solutions involving
    //neighbours in different classes, this will help reduce segmented area:
    penalty = params->smoothness_penalty_base;

    //INTENSITY SMOOTHNESS:
    int d_I           = im[p.x + psb_in*p.y] - im[np.x + psb_in*np.y];
    double sigma      = 2.0 * params->smoothness_3sigmaon2/3.0;
    double p_int_edge = 1.0 - exp(-(d_I*d_I)/(2.0*sigma*sigma));
    
    //If it's likely that it's an intensity edge, p_prob
    //return less additional penalty.
    //if it's not much of an edge, return 
    //more additional penalty:
    penalty += (int) (params->smoothness_penalty*(1.0-p_int_edge));
  }
 

  return penalty;
}

/* computes current energy */
int MultiClass::compute_energy() {
    //printf("computing energy \n");
    int   d,dq;
    Coord p,q;
    char c;
    
    //printf("variables %d \n", im_size.height);
 
    E = 0;
    for (p.y = 1; p.y < im_size.height-1; p.y++) {
        for (p.x = 1; p.x < im_size.width-1; p.x++) {
            //printf("%d %d %d %08x \n", p.y, p.x, psb, out);
            d = out[p.y * psb + p.x]; // d is supposed to be in the range between 0 and 1
            //non-neighbourhood terms:
            //printf("computing likelihood for d = %d \n", d);
            E += likelihood(p, d);
            //printf("success \n");
            
            for (int k=0; k<NEIGHBOR_NUM; k++) {
                q = p + NEIGHBORS[k];
                
                if (q>=Coord(1,1) && q<im_sz-Coord(1,1)) {
                    dq = out[q.y*psb + q.x];
                    //neighbourhood terms:
                    E += prior_intensity_smoothness(p, q, d, dq);
                }
            }
        }
    }
    
    return E;
    
}

void MultiClass::expand(int a)
{
	Coord p,q;
	int d,dq;
	Energy::Var var, qvar;
	int E_old, E00, E0a, Ea0;
	int k;

	printf("EXPANDING!!\n");

	Energy *e = new Energy(error_function);


	/* non-neighbourhood terms */
	for (p.y=1; p.y<im_size.height-1; p.y++)
		for (p.x=1; p.x<im_size.width-1; p.x++)
		{
			d = out[p.y*psb + p.x];
			if (a == d)
			{
				ptr_im[p.y*len_nv + p.x] = VAR_ACTIVE;
				e->add_constant(likelihood(p, d));
			}
			else
			{
				ptr_im[p.y*len_nv + p.x] = var = e->add_variable();
				e->ADD_TERM1(var, likelihood(p, d), likelihood(p, a));
			}
		}


		/* neighbourhood terms */
		for (p.y=1; p.y<im_size.height-1; p.y++)
			for (p.x=1; p.x<im_size.width-1; p.x++)
			{	
				d = out[p.y*psb + p.x];
				var = (Energy::Var) ptr_im[p.y*len_nv + p.x];
				for (k=0; k<NEIGHBOR_NUM; k++)
				{
					q = p + NEIGHBORS[k];
					if ( ! ( q>=Coord(1,1) && q<im_sz-Coord(1,1) ) ) continue;
					qvar = (Energy::Var) ptr_im[q.y*len_nv + q.x];
					dq = out[q.y*psb + q.x];
					if (var != VAR_ACTIVE && qvar != VAR_ACTIVE)
						E00 = prior_intensity_smoothness(p, q, d, dq);
					if (var != VAR_ACTIVE)
						E0a = prior_intensity_smoothness(p, q, d, a);
					if (qvar != VAR_ACTIVE)
						Ea0 = prior_intensity_smoothness(p, q, a, dq);
					if (var != VAR_ACTIVE)
					{
						if (qvar != VAR_ACTIVE) e->ADD_TERM2(var, qvar, E00, E0a, Ea0, 0);
						else                    e->ADD_TERM1(var, E0a, 0);
					}
					else
					{
						if (qvar != VAR_ACTIVE) e->ADD_TERM1(qvar, Ea0, 0);
					}
				}
			}

			E_old = E;
			E = e->minimize();

			printf("attempt to write a = %d \n",a);
			int tot = 0;
			int written = 0;
			if (E < E_old)
			{
				for (p.y=1; p.y<im_size.height-1; p.y++)
					for (p.x=1; p.x<im_size.width-1; p.x++)
					{
						tot++;
						var = (Energy::Var) ptr_im[p.y*len_nv + p.x];
						if (var!=VAR_ACTIVE && e->get_var(var)==VALUE1)
						{
							
							unsigned char temp = a;
							//printf("written %d  \n", temp);
							out[p.y*psb + p.x] = temp;
							written++;
							
						}
					}
			}

			printf("written %d / %d \n", written, tot);

			delete e;

}
