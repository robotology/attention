#include <math.h>
#include <fstream>
#include <cstdlib>
#include "darwin/segment.h"
#include "darwin/disjoint-set.h"


/*
Creates edges of grid graph with size (M,N). 
Returns array E of size (3,nE), where E[3*e+0] and E[3*e+1] are edge endpoints
and E[3*e+2] is horizontal/vertical flag. 
The edge ordering is compliant to TRW-S (i.e., monotonic chains).
*/
unsigned *grid_graph( int M, int N, int *nE )
{
  *nE = M*(N-1) + N*(M-1);
  unsigned *E0 = new unsigned[2*(*nE)], *E = E0;

  for ( int j=0; j<N; j++ ) {
    for ( int i=0; i<M-1; i++ ) {
      E[0] = i+0+M*j;
      E[1] = i+1+M*j;
      //E[2] = 0;
      E += 2;
    }
    if ( j<N-1 )
      for ( int i=0; i<M; i++ ) {
        E[0] = i+0+M*j;
        E[1] = i+M+M*j;
        //E[2] = 1;
        E += 2;
      }
  }

  return E0;
}


void compute_unary_potentials(
  const unsigned char *I, // array (3,nT), color image
  const real *W, // array (4,nK), parameters of SVM classifier
  int nK, // nr of labels (image segments)
  int nT, // nr of image pixels
  real *q // array (nK,nT) of unary costs (must be pre-allocated)
)
{
  for ( int t=0; t<nT; t++ ) {
    real *qt = q + nK*t;
    const unsigned char *It = I + 3*t;
    unsigned char minc=255, maxc=0;
    for ( int c=0; c<3; c++ ) {
      if ( It[c] < minc ) minc = It[c];
      if ( It[c] > maxc ) maxc = It[c];
    }
    for ( int k=0; k<nK; k++ ) {
      const real *Wk = W + 8*k;
      qt[k] = Wk[0]
            + (real)(Wk[1]*(real)It[0])
            + (real)(Wk[2]*(real)It[1])
            + (real)(Wk[3]*(real)It[2])
            + (real)(Wk[4]*((real)maxc-(real)minc))
            + (real)(Wk[5]*(real)It[1]*(real)It[2])
            + (real)(Wk[6]*(real)It[2]*(real)It[0])
            + (real)(Wk[7]*(real)It[0]*(real)It[1]);
    }
  }
}


void reparameterize_unary_potentials(
  const unsigned *E, // edges
  int nE, // nr of edges
  real *q, // array (nK,nT), unary potentials
  int nT, // nr of pixels
  int nK, // nr of labels
  const real *f // array (nK,2,nE), messages
)
{
  for ( int e=0; e<nE; e++ ) {
    real *qe = q + nK*E[2*e+0], *qee = q + nK*E[2*e+1];
	const real *fe = f + nK*(2*e+0), *fee = fe + nK;
    for ( int k=0; k<nK; k++ ) {
      qe[k] -= fe[k];
      qee[k] -= fee[k];
    }
  }
}


// MAP inference in the MRF by a message-passing algorithm (TRW-S).
// See [V.Kolmogorov: Convergent Tree-reweighted Message Passing for Energy Minimization].
real trws_potts( // returns residual
  const unsigned *E, // edges
  int nE, // nr of edges
  real w, // edge weights (non-negative)
  real *q, // array (nK,nT), unary potentials
  int nT, // nr of pixels
  int nK, // nr of labels
  real *f, // array (nK,2,nE), messages
  real gamma // TRW-S edge occurence probability (for grid graph, set gamma=1/2)
)
{
  real resid = 0, *d = new real[nK], INF = 1E10;

  // forward pass
  for ( int e=0; e<nE; e++ ) {
    real *qe = q + nK*E[0+e*2],
         *qee = q + nK*E[1+e*2],
         *fe = f + nK*(0+e*2),
         *fee = fe + nK;

    real a = -INF;
    for ( int k=0; k<nK; k++ ) {
      d[k] = gamma*qe[k] + fe[k];
      if ( d[k]>a ) a=d[k];
    }
    a -= w;

    real maxd = -INF;
    for ( int k=0; k<nK; k++ ) {
      if ( a>d[k] ) d[k]=a;
      d[k] += fee[k];
      if ( d[k]>maxd ) maxd=d[k];
    }

    for ( int k=0; k<nK; k++ ) {
      d[k] -= maxd;
      qee[k] += d[k];
      fee[k] -= d[k];
      resid += fabs(d[k]);
    }
  }

  // backward pass
  for ( int e=nE-1; e>=0; e-- ) {
    real *qe = q + nK*E[1+e*2],
         *qee = q + nK*E[0+e*2],
         *fe = f + nK*(1+e*2),
         *fee = fe - nK;

    real a = -INF;
    for ( int k=0; k<nK; k++ ) {
      d[k] = gamma*qe[k] + fe[k];
      if ( d[k]>a ) a=d[k];
    }
    a -= w;

    real maxd = -INF;
    for ( int k=0; k<nK; k++ ) {
      if ( a>d[k] ) d[k]=a;
      d[k] += fee[k];
      if ( d[k]>maxd ) maxd=d[k];
    }

    for ( int k=0; k<nK; k++ ) {
      d[k] -= maxd;
      qee[k] += d[k];
      fee[k] -= d[k];
      resid += fabs(d[k]);
    }

  }

  delete[] d;
  return resid/(nK*2*nE); // normalize residual to one label*pixel
}


void extract_labeling(
  const real *q, // array (nK,nT), unary potentials
  int nK, // nr of labels
  int nT, // nr of pixels
  unsigned char *K // nT-vector, labeling (must be pre-allocated)
)
{
  int diff = 0;
  for ( int t=0; t<nT; t++ ) {
    const real *qt = q + nK*t;
    unsigned char *Kt = K + t;
    real maxq = qt[0];
    *Kt = 0;
    for ( int k=1; k<nK; k++ )
      if ( qt[k]>maxq ) {
        maxq = qt[k];
        *Kt = k;
      }
  }
}


/*
Multilabel connected components.
Assigns different label to each connected region of non-background pixels.
Background has label 0.
The algorithm: the classical binary connected component labeling,
modified for multiple labels in one-against-all manner.
*/
unsigned connected_components( // returns the number of components excl. background (=max(J))
  const unsigned char *I,  // input image (IMPORTANT: values in row 0 and column 0 must equal 0)
  int width, int height,   // image dimensions
  unsigned minsize,        // minimal size of a component
  unsigned *J              // output image
)
{
  unsigned nlabels = 0;
  for ( int i=0; i<width*height; i++ ) {
    J[i] = (I[i] > 0);
    if ( I[i] > nlabels ) nlabels = I[i];
  }

  const unsigned MAXLABEL = 10*(width+height); // increase if components are very ragged

  universe U((nlabels+1)*MAXLABEL);
  unsigned *newlabel = new unsigned[nlabels+1];
  for ( int k=0; k<=nlabels; k++ ) newlabel[k] = 0;

  for ( int j=1, jj=width; j<height; j++, jj+=width )
    for ( int i=1; i<width; i++ ) {
      int ij = i+jj;
      unsigned J0 = J[ij];
      if ( J0>0 ) {
        unsigned J1 = J[ij-1], J2 = J[ij-width];
        unsigned char I0 = I[ij], I1 = I[ij-1], I2 = I[ij-width];
        if ( I1 != I0 ) {
          if ( I2 != I0 ) {
            J0 = ++newlabel[I0];
            if ( J0 >= MAXLABEL ) {
              fprintf(stderr,"Error: MAXLABEL exceeded, increase it.\n");
              exit(1);
            }
          } else J0 = J2;
        } else {
          if ( I2 != I0 ) J0 = J1;
          else {
            J0 = J1;
            if ( J1 != J2 ) U.join( U.find(J1+I0*MAXLABEL), U.find(J2+I0*MAXLABEL) );
          }
        }
      }
      J[i+width*j] = J0;
    }

//  printf("max labels = [");
//  for ( int k=0; k<=nlabels; k++ ) printf("%i ",(int)newlabel[k]);
//  printf("]\n");

  // Transform J to representants of equivalence classes for each component.
  // Compute n[k] the number of pixels with label k.
  unsigned *n = new unsigned[U.nelem()];
  for ( int k=0; k<U.nelem(); k++ ) n[k] = 0;
  for ( int i=0; i<width*height; i++ )
    if ( J[i] ) {
      J[i] = U.find( J[i]+I[i]*MAXLABEL );
      n[J[i]]++;
    }

  //for ( int k=0; k<U.nelem(); k++ ) if (n[k]) printf("%i ",k);

  // Compute total number of components (with size thresholded by minsize).
  // Re-index J so that its values are in range 0..(ncomponents-1) without gaps.
  unsigned ncomponents = 1;
  for ( int k=0; k<U.nelem(); k++ ) n[k] = (n[k]>minsize ? ncomponents++ : 0);
  for ( int i=0; i<width*height; i++ ) if ( J[i] ) J[i] = n[J[i]];
  ncomponents--;

  delete [] n;
  delete [] newlabel;

  return ncomponents;
}


/*
Computes bounding boxes of components labeled by a single label in J.
To each bounding box is assigned the corresponding label from I.
*/
void bounding_boxes(
  const unsigned *J,
  const unsigned char *I,
  int width, int height,
  int ncomponents,
  int *bbox         // array of length 5*max(I) with tuples [minrow maxrow mincol maxcol label] 
)
{
  for ( int k=0; k<ncomponents; k++ ) {
    int *b = bbox + 5*k;
    b[0] = width;
    b[1] = -1;
    b[2] = height;
    b[3] = -1;
  }
  for ( int j=1, jj=width; j<height; j++, jj+=width )
    for ( int i=1; i<width; i++ ) {
      int ij = i+jj;
      unsigned k = J[ij];
      if ( k ) {
        int *b = bbox + 5*(k-1);
        if ( i<b[0] ) { b[0]=i; b[4]=I[ij]; }
        if ( i>b[1] ) b[1]=i;
        if ( j<b[2] ) b[2]=j;
        if ( j>b[3] ) b[3]=j;
      }
    }
}
