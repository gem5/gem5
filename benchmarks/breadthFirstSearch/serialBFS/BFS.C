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

#include "graph.h"
#include "utils.h"
#include "parallel.h"
using namespace std;

// **************************************************************
//    SERIAL BREADTH FIRST SEARCH
// **************************************************************

// **************************************************************
//    THE SERIAL BSF
//    Updates the graph so that it is the BFS tree (i.e. the neighbors
//      in the new graph are the children in the bfs tree)
// **************************************************************

pair<intT,intT> BFS(intT start, graph<intT> GA) {
  intT numVertices = GA.n;
  intT numEdges = GA.m;
  vertex<intT> *G = GA.V;
  intT* Frontier = newA(intT,numEdges);
  intT* Visited = newA(intT,numVertices);
  for (intT i = 0; i < numVertices; i++)
    Visited[i] = 0;

  intT bot = 0;
  intT top = 1;
  Frontier[0] = start;
  Visited[start] = 1;

  while (top > bot) {
    intT v = Frontier[bot++];
    intT k = 0;
    for (intT j=0; j < G[v].degree; j++) {
      intT ngh = G[v].Neighbors[j];
      if (Visited[ngh] == 0) {
	Frontier[top++] = G[v].Neighbors[k++] = ngh;
	Visited[ngh] = 1;
      }
    }
    G[v].degree = k;
  }

  free(Frontier); free(Visited);
  return pair<intT,intT>(0,0);
}
