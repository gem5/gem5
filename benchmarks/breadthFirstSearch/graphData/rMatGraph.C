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

template <class intT>
struct rMat {
  double a, ab, abc;
  intT n;
  intT h;
  rMat(intT _n, intT _seed,
       double _a, double _b, double _c) {
    n = _n; a = _a; ab = _a + _b; abc = _a+_b+_c;
    h = hash<uintT>(_seed);
    utils::myAssert(abc <= 1.0,
		    "in rMat: a + b + c add to more than 1");
    utils::myAssert((1 << utils::log2Up(n)) == n,
		    "in rMat: n not a power of 2");
  }

  edge<intT> rMatRec(intT nn, intT randStart, intT randStride) {
    if (nn==1) return edge<intT>(0,0);
    else {
      edge<intT> x = rMatRec(nn/2, randStart + randStride, randStride);
      double r = hash<double>(randStart);
      if (r < a) return x;
      else if (r < ab) return edge<intT>(x.u,x.v+nn/2);
      else if (r < abc) return edge<intT>(x.u+nn/2, x.v);
      else return edge<intT>(x.u+nn/2, x.v+nn/2);
    }
  }

  edge<intT> operator() (intT i) {
    intT randStart = hash<uintT>((2*i)*h);
    intT randStride = hash<uintT>((2*i+1)*h);
    return rMatRec(n, randStart, randStride);
  }
};

template <class intT>
edgeArray<intT> edgeRmat(intT n, intT m, intT seed,
		   float a, float b, float c) {
  intT nn = (1 << utils::log2Up(n));
  rMat<intT> g(nn,seed,a,b,c);
  edge<intT>* E = newA(edge<intT>,m);
  parallel_for (intT i = 0; i < m; i++)
    E[i] = g(i);
  return edgeArray<intT>(E,nn,nn,m);
}


int parallel_main(int argc, char* argv[]) {
  commandLine P(argc,argv,
		"[-m <numedges>] [-s <intseed>] [-o] [-j] [-a <a>] [-b <b>] [-c <c>] n <outFile>");
  pair<intT,char*> in = P.sizeAndFileName();
  intT n = in.first;
  char* fname = in.second;
  double a = P.getOptionDoubleValue("-a",.5);
  double b = P.getOptionDoubleValue("-b",.1);
  double c = P.getOptionDoubleValue("-c", b);
  intT m = P.getOptionLongValue("-m", 10*n);
  intT seed = P.getOptionLongValue("-s", 1);
  bool adjArray = P.getOption("-j");
  bool ordered = P.getOption("-o");

  edgeArray<intT> EA = edgeRmat(n, m, seed, a, b, c);
  if (!ordered) {
    graph<intT> G = graphFromEdges<intT>(EA,adjArray);
    EA.del();
    G = graphReorder<intT>(G, NULL);
    if (adjArray) {
      writeGraphToFile<intT>(G, fname);
      G.del();
    } else {
      EA = edgesFromGraph<intT>(G);
      G.del();
      std::random_shuffle(EA.E, EA.E + m);
      writeEdgeArrayToFile<intT>(EA, fname);
      EA.del();
    }
  } else {
    if (adjArray) {
      graph<intT> G = graphFromEdges<intT>(EA, 1);
      EA.del();
      writeGraphToFile<intT>(G, fname);
      G.del();
    } else {
      writeEdgeArrayToFile<intT>(EA, fname);
      EA.del();
    }
  }
}
