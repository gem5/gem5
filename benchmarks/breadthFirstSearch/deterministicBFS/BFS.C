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

#include "utils.h"
#include "sequence.h"
#include "graph.h"
#include "parallel.h"
#include <limits.h>
using namespace std;

// **************************************************************
//    DETERMINISTIC BREADTH FIRST SEARCH
// **************************************************************

// **************************************************************
//    THE DETERMINISTIC BSF
//    Updates the graph so that it is the BFS tree (i.e. the neighbors
//      in the new graph are the children in the bfs tree)
// **************************************************************

struct nonNegF{bool operator() (intT a) {return (a>=0);}};

pair<intT,intT> BFS(intT start, graph<intT> GA) {
  intT numVertices = GA.n;
  intT numEdges = GA.m;
  vertex<intT> *G = GA.V;
  intT* Offsets = newA(intT,numVertices+1);
  intT* Parents = newA(intT,numVertices);
  parallel_for (intT i = 0; i < numVertices; i++) Parents[i] = INT_MAX;
  intT* Frontier = newA(intT,numVertices);
  intT* FrontierNext = newA(intT,numEdges);

  Frontier[0] = start;
  intT fSize = 1;
  Parents[start] = -1;
  int round = 0;
  intT totalVisited = 0;

  while (fSize > 0) {
    totalVisited += fSize;
    round++;

    // For each vertex in the frontier try to "hook" unvisited neighbors.
    parallel_for(intT i = 0; i < fSize; i++) {
      intT k= 0;
      intT v = Frontier[i];
      for (intT j=0; j < G[v].degree; j++) {
        intT ngh = G[v].Neighbors[j];
	if (Parents[ngh] > v)
	  if (utils::writeMin(&Parents[ngh],v))
	    G[v].Neighbors[k++] = ngh;
      }
      Offsets[i] = k;
    }

    // Find offsets to write the next frontier for each v in this frontier
    intT nr=sequence::scan(Offsets,Offsets,fSize,utils::addF<intT>(),(intT)0);
    Offsets[fSize] = nr;

    // Move hooked neighbors to next frontier.
    parallel_for (intT i = 0; i < fSize; i++) {
      intT o = Offsets[i];
      intT d = Offsets[i+1]-o;
      intT k = 0;
      intT v = Frontier[i];
      for (intT j=0; j < d; j++) {
	intT ngh = G[v].Neighbors[j];
	if (Parents[ngh] == v) {
	  FrontierNext[o+j] = G[v].Neighbors[k++] = ngh;
	  Parents[ngh] = -1;
	}
	else FrontierNext[o+j] = -1;
      }
      G[v].degree = k;
    }

    // Filter out the empty slots (marked with -1)
    fSize = sequence::filter(FrontierNext, Frontier, nr, nonNegF());
  }

  free(FrontierNext); free(Frontier); free(Offsets); free(Parents);

  return pair<intT,intT>(totalVisited,round);
}
