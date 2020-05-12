/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#ifndef DISJOINT_SET
#define DISJOINT_SET

// disjoint-set forests using union-by-rank and path compression (sort of).

typedef struct {
  int rank;
  int p;
  //int size;
} uni_elt;

class universe {
public:
  universe(int elements);
  ~universe();
  int find(int x);  
  void join(int x, int y);
  //int size(int x) const { return elts[x].size; }
  //int num_sets() const { return num; }
  int nelem() const { return num; }

private:
  uni_elt *elts;
  int num;
};

universe::universe(int elements) {
  elts = new uni_elt[elements];
  num = elements;
  for (int i = 0; i < elements; i++) {
    elts[i].rank = 0;
    //elts[i].size = 1;
    elts[i].p = i;
  }
}
  
universe::~universe() {
  delete [] elts;
}

int universe::find(int x) {
  //if ( (x<0) || (x>=num) ) fprintf(stderr,"Error: weird argument in universe::find.\n");
  int y = x;
  while (y != elts[y].p)
    y = elts[y].p;
  elts[x].p = y;
  //if ( (y<0) || (y>=num) ) fprintf(stderr,"Error: weird return value in universe::find.\n");
  return y;
}

void universe::join(int x, int y) {
  //if ( (x<0) || (x>=num) || (y<0) || (y>=num) ) fprintf(stderr,"Error: weird argument in universe::join.\n");
  if (elts[x].rank > elts[y].rank) {
    elts[y].p = x;
    //printf("%i ",elts[y].size);
    //elts[x].size += elts[y].size;
  } else {
    elts[x].p = y;
    //printf("%i ",elts[x].size);
    //elts[y].size += elts[x].size;
    if (elts[x].rank == elts[y].rank)
      elts[y].rank++;
  }
  //num--;
}

#endif
