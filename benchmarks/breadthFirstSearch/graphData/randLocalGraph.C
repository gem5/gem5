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

#include "IO.h"
#include "parseCommandLine.h"
#include "graph.h"
#include "graphIO.h"
#include "dataGen.h"
#include "graphUtils.h"
#include "parallel.h"
using namespace benchIO;
using namespace dataGen;
using namespace std;

// Generates an undirected graph with n vertices with approximately degree
// neighbors per vertex.
// Edges  are distributed so they appear to come from
// a dim-dimensional space.   In particular an edge (i,j) will have
// probability roughly proportional to (1/|i-j|)^{(d+1)/d}, giving
// separators of size about n^{(d-1)/d}.
template <class intT>
edgeArray<intT> edgeRandomWithDimension(intT dim, intT degree, intT numRows) {
  intT nonZeros = numRows*degree;
  edge<intT> *E = newA(edge<intT>,nonZeros);
  parallel_for (intT k=0; k < nonZeros; k++) {
    intT i = k / degree;
    intT j;
    if (dim==0) {
      intT h = k;
      do {
	j = ((h = hash<intT>(h)) % numRows);
      } while (j == i);
    } else {
      intT pow = dim+2;
      intT h = k;
      do {
	while ((((h = hash<intT>(h)) % 1000003) < 500001)) pow += dim;
	j = (i + ((h = hash<intT>(h)) % (((long) 1) << pow))) % numRows;
      } while (j == i);
    }
    E[k].u = i;  E[k].v = j;
  }
  return edgeArray<intT>(E,numRows,numRows,nonZeros);
}

int parallel_main(int argc, char* argv[]) {
  commandLine P(argc,argv,"[-m <numedges>] [-d <dims>] [-o] [-j] n <outFile>");
  pair<intT,char*> in = P.sizeAndFileName();
  intT n = in.first;
  char* fname = in.second;
  int dim = P.getOptionIntValue("-d", 0);
  intT m = P.getOptionLongValue("-m", 10*n);
  bool ordered = P.getOption("-o");
  bool adjArray = P.getOption("-j");
  edgeArray<intT> EA = edgeRandomWithDimension<intT>(dim, m/n, n);
  int r;
  if (adjArray) {
    graph<intT> G = graphFromEdges<intT>(EA,1);
    EA.del();
    if (!ordered) G = graphReorder<intT>(G, NULL);
    r = writeGraphToFile<intT>(G, fname);
    G.del();
  } else {
    if (!ordered) std::random_shuffle(EA.E, EA.E + EA.nonZeros);
    r = writeEdgeArrayToFile<intT>(EA, fname);
    EA.del();
  }
  return r;
}
