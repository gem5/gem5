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

#include <iostream>
#include <algorithm>
#include <cstring>
#include "parallel.h"
#include "IO.h"
#include "graph.h"
#include "graphIO.h"
#include "parseCommandLine.h"
using namespace std;
using namespace benchIO;

intT levelNumber(intT start, intT level, intT* P, intT* L, graph<intT> T) {
  if (L[start] != -1) {
    cout << "BFSCheck: not a tree" << endl;
    return 1;
  }
  L[start] = level;
  for (intT i=0; i < T.V[start].degree; i++) {
    intT n = T.V[start].Neighbors[i];
    P[n] = start;
    levelNumber(n, level+1, P, L, T);
  }
  return 0;
}

// Checks if T is valid BFS tree relative to G starting at i
int checkBFS(intT start, graph<intT> G, graph<intT> T) {
  if (G.n != T.n) {
    cout << "BFSCheck: vertex counts don't match: " << G.n << ", " << T.n << endl;
    return 1;
  }
  if (T.m > G.n - 1) {
    cout << "BFSCheck: too many edges in tree " << endl;
    return 1;
  }
  intT* P = newA(intT, G.n);
  intT* L = newA(intT, G.n);
  parallel_for (intT i=0; i < G.n; i++) {
    P[i] = -1;
    L[i] = -1;
  }
  if (levelNumber(start, 0, P, L, T)) return 1;
  for (intT i=0; i < G.n; i++) {
    bool Check=0;
    if (L[i] == -1) {
      for (intT j=0; j < G.V[i].degree; j++) {
	intT ngh = G.V[i].Neighbors[j];
	if (L[ngh] != -1) {
	  cout << "BFSCheck: connected vertex not in tree " << endl;
	  return 1;
	}
      }
    } else {
      for (intT j=0; j < G.V[i].degree; j++) {
	intT ngh = G.V[i].Neighbors[j];
	if (P[i] == ngh) Check = 1;
	else if (L[ngh] > L[i] + 1 || L[ngh] < L[i] - 1) {
	  cout << "BFSCheck: edge spans two levels " << endl;
	  return 1;
	}
      }
      if (i != start && Check == 0) {
	cout << "BFSCheck: parent not an edge " << endl;
	return 1;
      }
    }
  }
  return 0;
}

int parallel_main(int argc, char* argv[]) {
  commandLine P(argc,argv,"<inFile> <outfile>");
  pair<char*,char*> fnames = P.IOFileNames();
  char* iFile = fnames.first;
  char* oFile = fnames.second;

  graph<intT> G = readGraphFromFile<intT>(iFile);
  graph<intT> T = readGraphFromFile<intT>(oFile);

  return checkBFS(0, G, T);
}
