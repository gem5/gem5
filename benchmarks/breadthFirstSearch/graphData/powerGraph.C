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
#include <vector>
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

static float powerlaw_random(float dmin, float dmax, float n) {
  float r = (float) random() / RAND_MAX;
  return pow((pow(dmax, n) - pow(dmin, n)) * pow(r, 3) + pow(dmin, n), 1.0 / n);
}

vector<int> siteSizes(intT n) {
  static vector<int> site_sizes;
  srand(0);
  for (int i = 0; i < n;) {
    int c = powerlaw_random(1,
                            min(50000, (int) (100000. * n / 100e6)),
                            0.001);
    c = (c==0)?1:c;
    site_sizes.push_back(c);
    i += c;
  }
  return site_sizes;
}

graph<intT> makePowerGraph(intT n) {
  intT inRatio = 10;
  intT degree = 15;
  intT *ngh = newA(intT, degree * n);
  vertex<intT> *V = newA(vertex<intT>, n);
  parallel_for(intT i=0; i < n; i++) {
    V[i].degree = degree;
    V[i].Neighbors = ngh + i * degree;
  }

  graph<intT> G = graph<intT>(V, n, degree*n, ngh);

  vector<int> site_sizes = siteSizes(n);
  intT sites = site_sizes.size();
  intT *sizes = newA(intT, sites);
  intT *offsets = newA(intT, sites);
  intT o = 0;
  for (intT i=0; i < sites; i++) {
    sizes[i] = site_sizes[i];
    offsets[i] = o;
    o += sizes[i];
    if (o > n) sizes[i] = max<intT>(0,sizes[i] + n - o);
  }

  for (intT i=0; i < sites; i++) {
    for (intT j =0; j < sizes[i]; j++) {
      for (intT k = 0; k < degree; k++) {
	intT target_site = (random() % inRatio != 0) ? i : (random() % sites);
	intT site_id = random() % sizes[target_site];
	intT idx = offsets[i] + j;
	G.V[idx].Neighbors[k] = offsets[target_site] + site_id;
      }
    }
  }
  return G;
}

int parallel_main(int argc, char* argv[]) {
  commandLine P(argc,argv,"[-j] [-o] n <outFile>");
  pair<int,char*> in = P.sizeAndFileName();
  intT n = in.first;
  char* fname = in.second;
  bool ordered = P.getOption("-o");
  bool adjArray = P.getOption("-j");
  edgeArray<intT> EA;
  graph<intT> G = makePowerGraph(n);
  if (adjArray) {
    if (!ordered) G = graphReorder<intT>(G, NULL);
    int r = writeGraphToFile<intT>(G, fname);
    G.del();
    return r;
  } else {
    edgeArray<intT> EA = edgesFromGraph(G);
    G.del();
    if (!ordered) std::random_shuffle(EA.E, EA.E + EA.nonZeros);
    int r = writeEdgeArrayToFile<intT>(EA, fname);
    EA.del();
    return r;
  }
}
