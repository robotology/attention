#include <fstream>
#include <cstdlib>
#include <limits.h>
#include <cstring>

using namespace std;
/* Utilities for loading and saving a numeric matrix (array) in text format. */

/* Read a numeric array (matrix) of type T from file. */
template <class T>
void load_numeric_array( const char *name, int *nrows, int *ncols, T **X )
{
  FILE *fid;
  
  fid = fopen(name,"r");
  if ( !fid ) {
    fprintf(stderr,"Error: Cannot open file <%s>.\n",name);
    exit(1);
  }
  *ncols = *nrows = 0;
  int BOF = 1;
  while (1) {
    int c;
    do c = getc(fid); while ( c==' ' );
    if ( feof(fid) )
	  break;
    else if ( c=='\n' ) BOF=1;
    else {
      ungetc(c,fid);
      double x;
      int n = fscanf(fid,"%lg",&x);
      if ( BOF ) { (*nrows)++; BOF=0; }
      (*ncols)++;
    }
  }
  fclose(fid);

  // Second pass: allocate X and read its values
  *X = new T[*ncols];
  fid = fopen(name,"r");
  for ( int i=0; i<*ncols; i++ ) {
    double x;
    int n = fscanf(fid,"%lg",&x);
    (*X)[i] = (T)x;
  }
  fclose(fid);

  *ncols = (*ncols) / (*nrows);
}


/* Saves a numeric array (matrix) of type T to file. */
template <class T>
void save_numeric_array( const char *name, int nrows, int ncols, T *X )
{
  FILE *fid = fopen(name,"w");
  if ( !fid ) {
    fprintf(stderr,"Error: Cannot open file <%s>.\n",name);
    exit(1);
  }
  int n = 0;
  for ( int i=0; i<nrows; i++ ) {
    for ( int j=0; j<ncols; j++ ) {
      double x = X[n++];
      fprintf(fid,"%lg ",x);
    }
    fprintf(fid,"\n");
  }
  fclose(fid);
}


/* The following functions are adapted from pnmfile.h by Pedro Felzenszwalb. */

/* read PNM field, skipping comments */ 
static void pnm_read(std::ifstream &file, char *buf) {
  const int BUF_SIZE=256;
  char doc[BUF_SIZE];
  char c;
  
  file >> c;
  while (c == '#') {
    file.getline(doc, BUF_SIZE);
    file >> c;
  }
  file.putback(c);
  
  file.width(BUF_SIZE);
  file >> buf;
  file.ignore();
}


unsigned char *loadPGM(const char *name, int *width, int *height)
{
  const int BUF_SIZE=256;
  char buf[BUF_SIZE];
  
  /* read header */
  std::ifstream file(name, std::ios::in | std::ios::binary);
  if (!file) {
    fprintf(stderr,"Error: Cannot open file %s, exiting.\n",name);
    exit(1);
  }
  pnm_read(file, buf);
  if (strncmp(buf, "P5", 2)) {
    fprintf(stderr,"Error: Not a PGM format, exiting.\n");
    exit(1);
  }

  pnm_read(file, buf);
  *width = atoi(buf);
  pnm_read(file, buf);
  *height = atoi(buf);

  pnm_read(file, buf);
  if (atoi(buf) > UCHAR_MAX) {
    fprintf(stderr,"Error: PGM with byte size greater than %i not supported, exiting.\n",UCHAR_MAX);
    exit(1);
  }

  /* read data */
  unsigned char *im = new unsigned char[sizeof(char)*(*width)*(*height)];
  file.read((char*)im,sizeof(char)*(*width)*(*height));

  return im;
}


void savePGM(const char *name, const int width, const int height, const unsigned char *im)
{
  std::ofstream file(name, std::ios::out | std::ios::binary);

  file << "P5\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
  file.write((char*)im, sizeof(char)*width*height);
}


unsigned char *loadPPM(const char *name, int *width, int *height)
{
  const int BUF_SIZE=256;
  char buf[BUF_SIZE];
  
  /* read header */
  std::ifstream file(name, std::ios::in | std::ios::binary);
  if (!file) {
    fprintf(stderr,"Error: Cannot open file %s, exiting.\n",name);
    exit(1);
  }
  pnm_read(file, buf);
  if (strncmp(buf, "P6", 2)) {
    fprintf(stderr,"Error: Not a PPM format, exiting.\n");
    exit(1);
  }

  pnm_read(file, buf);
  *width = atoi(buf);
  pnm_read(file, buf);
  *height = atoi(buf);

  pnm_read(file, buf);
  if (atoi(buf) > UCHAR_MAX) {
    fprintf(stderr,"Error: PPM with byte size greater than %i not supported, exiting.\n",UCHAR_MAX);
    exit(1);
  }

  unsigned char *im = new unsigned char[3*(*width)*(*height)];
  file.read((char*)im,3*(*width)*(*height));
  file.close();
  return im;
}


void savePPM(const char *name, const int width, const int height, const unsigned char *im)
{
  std::ofstream file(name, std::ios::out | std::ios::binary);

  file << "P6\n" << width << " " << height << "\n" << UCHAR_MAX << "\n";
  file.write((char*)im, width*height*3);
  file.close();
}
