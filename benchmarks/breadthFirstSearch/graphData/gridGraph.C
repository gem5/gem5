// This code is part of the Problem Based Benchmark Suite (PBBS)
// Copyright (c) 2011 Guy Blelloch and the PBBS team
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights (to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <math.h>
#include "IO.h"
#include "parseCommandLine.h"
#include "graph.h"
#include "graphIO.h"
#include "graphUtils.h"
#include "dataGen.h"
#include "parallel.h"
using namespace benchIO;
using namespace dataGen;
using namespace std;

template <class intT>
intT loc2d(intT n, intT i1, intT i2) {
  return ((i1 + n) % n)*n + (i2 + n) % n;
}

template <class intT>
edgeArray<intT> edge2DMesh(intT n) {
  intT dn = round(pow((float) n,1.0/2.0));
  intT nn = dn*dn;
  intT nonZeros = 2*nn;
  edge<intT> *E = newA(edge<intT>,nonZeros);
  parallel_for (intT i=0; i < dn; i++)
    for (intT j=0; j < dn; j++) {
      intT l = loc2d(dn,i,j);
      E[2*l] = edge<intT>(l,loc2d(dn,i+1,j));
      E[2*l+1] = edge<intT>(l,loc2d(dn,i,j+1));
    }
  return edgeArray<intT>(E,nn,nn,nonZeros);
}

template <class intT>
intT loc3d(intT n, intT i1, intT i2, intT i3) {
  return ((i1 + n) % n)*n*n + ((i2 + n) % n)*n + (i3 + n) % n;
}

template <class intT>
edgeArray<intT> edge3DMesh(intT n) {
  intT dn = round(pow((float) n,1.0/3.0));
  intT nn = dn*dn*dn;
  intT nonZeros = 3*nn;
  edge<intT> *E = newA(edge<intT>,nonZeros);
  parallel_for (intT i=0; i < dn; i++)
    for (intT j=0; j < dn; j++)
      for (intT k=0; k < dn; k++) {
	intT l = loc3d(dn,i,j,k);
	E[3*l] =   edge<intT>(l,loc3d(dn,i+1,j,k));
	E[3*l+1] = edge<intT>(l,loc3d(dn,i,j+1,k));
	E[3*l+2] = edge<intT>(l,loc3d(dn,i,j,k+1));
      }
  return edgeArray<intT>(E,nn,nn,nonZeros);
}


int parallel_main(int argc, char* argv[]) {
  commandLine P(argc,argv,"[-d {2,3}] [-j] [-o] n <outFile>");
  pair<int,char*> in = P.sizeAndFileName();
  intT n = in.first;
  char* fname = in.second;
  int dims = P.getOptionIntValue("-d", 2);
  bool ordered = P.getOption("-o");
  bool adjArray = P.getOption("-j");
  edgeArray<intT> EA;
  if (dims == 2)
    EA = edge2DMesh(n);
  else if (dims == 3)
    EA = edge3DMesh(n);
  else
    P.badArgument();
  if (adjArray) {
    graph<intT> G = graphFromEdges<intT>(EA,1);
    EA.del();
    if (!ordered) G = graphReorder<intT>(G, NULL);
    int r = writeGraphToFile<intT>(G, fname);
    G.del();
    return r;
  } else {
    if (!ordered) std::random_shuffle(EA.E, EA.E + EA.nonZeros);
    int r = writeEdgeArrayToFile<intT>(EA, fname);
    EA.del();
    return r;
  }
}
