/*
This is a library for color image segmentation.
 */

// The arithmetic in which MRF optimization is done.
typedef float real;

void compute_unary_potentials(
  const unsigned char *,
  const float *,
  int,
  int,
  real *
);

void reparameterize_unary_potentials(
  const unsigned *,
  int,
  real *,
  int,
  int,
  const real *
);

unsigned *grid_graph(
  int,
  int,
  int *
);

float trws_potts(
  const unsigned *,
  int,
  real,
  real *,
  int,
  int,
  real *,
  real
);

void extract_labeling(
  const real *,
  int,
  int,
  unsigned char *
);

unsigned connected_components(
  const unsigned char *,
  int, int,
  unsigned,
  unsigned *
);

void bounding_boxes(
  const unsigned *,
  const unsigned char *,
  int, int,
  int,
  int *
);
