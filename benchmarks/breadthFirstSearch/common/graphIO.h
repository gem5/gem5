// This code is part of the Problem Based Benchmark Suite (PBBS)
// Copyright (c) 2010 Guy Blelloch and the PBBS team
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

#ifndef _BENCH_GRAPH_IO
#define _BENCH_GRAPH_IO

#include "parallel.h"
#include "IO.h"

using namespace benchIO;

template <class intT>
int xToStringLen(edge<intT> a) {
  return xToStringLen(a.u) + xToStringLen(a.v) + 1;
}

template <class intT>
void xToString(char* s, edge<intT> a) {
  int l = xToStringLen(a.u);
  xToString(s, a.u);
  s[l] = ' ';
  xToString(s+l+1, a.v);
}

template <class intT>
int xToStringLen(wghEdge<intT> a) {
  return xToStringLen(a.u) + xToStringLen(a.v) + xToStringLen(a.weight) + 2;
}

template <class intT>
void xToString(char* s, wghEdge<intT> a) {
  int lu = xToStringLen(a.u);
  int lv = xToStringLen(a.v);
  xToString(s, a.u);
  s[lu] = ' ';
  xToString(s+lu+1, a.v);
  s[lu+lv+1] = ' ';
  xToString(s+lu+lv+2, a.weight);
}

namespace benchIO {
  using namespace std;

  string AdjGraphHeader = "AdjacencyGraph";
  string EdgeArrayHeader = "EdgeArray";
  string WghEdgeArrayHeader = "WeightedEdgeArray";

  template <class intT>
  int writeGraphToFile(graph<intT> G, char* fname) {
    intT m = G.m;
    intT n = G.n;
    intT totalLen = 2 + n + m;
    intT *Out = newA(intT, totalLen);
    Out[0] = n;
    Out[1] = m;
    parallel_for (intT i=0; i < n; i++) {
      Out[i+2] = G.V[i].degree;
    }
    intT total = sequence::scan(Out+2,Out+2,n,utils::addF<intT>(),(intT)0);
    for (intT i=0; i < n; i++) {
      intT *O = Out + (2 + n + Out[i+2]);
      vertex<intT> v = G.V[i];
      for (intT j = 0; j < v.degree; j++)
	O[j] = v.Neighbors[j];
    }
    int r = writeArrayToFile(AdjGraphHeader, Out, totalLen, fname);
    free(Out);
    return r;
  }

  template <class intT>
  int writeEdgeArrayToFile(edgeArray<intT> EA, char* fname) {
    intT m = EA.nonZeros;
    int r = writeArrayToFile(EdgeArrayHeader, EA.E, m, fname);
    return r;
  }

  template <class intT>
  int writeWghEdgeArrayToFile(wghEdgeArray<intT> EA, char* fname) {
    intT m = EA.m;
    int r = writeArrayToFile(WghEdgeArrayHeader, EA.E, m, fname);
    return r;
  }

  template <class intT>
  edgeArray<intT> readEdgeArrayFromFile(char* fname) {
    _seq<char> S = readStringFromFile(fname);
    words W = stringToWords(S.A, S.n);
    if (W.Strings[0] != EdgeArrayHeader) {
      cout << "Bad input file" << endl;
      abort();
    }
    intT n = (W.m-1)/2;
    edge<intT> *E = newA(edge<intT>,n);
    {parallel_for(intT i=0; i < n; i++)
      E[i] = edge<intT>(atol(W.Strings[2*i + 1]),
		  atol(W.Strings[2*i + 2]));}
    //W.del(); // to deal with performance bug in malloc

    intT maxR = 0;
    intT maxC = 0;
    for (intT i=0; i < n; i++) {
      maxR = max<intT>(maxR, E[i].u);
      maxC = max<intT>(maxC, E[i].v);
    }
    return edgeArray<intT>(E, maxR+1, maxC+1, n);
  }

  template <class intT>
  wghEdgeArray<intT> readWghEdgeArrayFromFile(char* fname) {
    _seq<char> S = readStringFromFile(fname);
    words W = stringToWords(S.A, S.n);
    if (W.Strings[0] != WghEdgeArrayHeader) {
      cout << "Bad input file" << endl;
      abort();
    }
    intT n = (W.m-1)/3;
    wghEdge<intT> *E = newA(wghEdge<intT>,n);
    {parallel_for(intT i=0; i < n; i++)
      E[i] = wghEdge<intT>(atol(W.Strings[3*i + 1]),
			   atol(W.Strings[3*i + 2]),
			   atof(W.Strings[3*i + 3]));}
    //W.del(); // to deal with performance bug in malloc

    intT maxR = 0;
    intT maxC = 0;
    for (intT i=0; i < n; i++) {
      maxR = max<intT>(maxR, E[i].u);
      maxC = max<intT>(maxC, E[i].v);
    }
    return wghEdgeArray<intT>(E, max<intT>(maxR,maxC)+1, n);
  }

  template <class intT>
  graph<intT> readGraphFromFile(char* fname) {
    _seq<char> S = readStringFromFile(fname);
    words W = stringToWords(S.A, S.n);
    if (W.Strings[0] != AdjGraphHeader) {
      cout << "Bad input file" << endl;
      abort();
    }
    intT len = W.m -1;
    intT * In = newA(intT, len);
    {parallel_for(intT i=0; i < len; i++) In[i] = atol(W.Strings[i + 1]);}
    //W.del(); // to deal with performance bug in malloc

    intT n = In[0];
    intT m = In[1];
    if (len != n + m + 2) {
      cout << "Bad input file" << endl;
      abort();
    }
    vertex<intT> *v = newA(vertex<intT>,n);
    intT* offsets = In+2;
    intT* edges = In+2+n;
    parallel_for (intT i=0; i < n; i++) {
      intT o = offsets[i];
      intT l = ((i == n-1) ? m : offsets[i+1])-offsets[i];
      v[i].degree = l;
      v[i].Neighbors = edges+o;
    }
    return graph<intT>(v,n,m,In);
  }

};

#endif // _BENCH_GRAPH_IO
