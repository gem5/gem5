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

#include <stdint.h>
#include <string>

#include <math.h>
#include "IO.h"
#include "parseCommandLine.h"
#include "graph.h"
#include "graphIO.h"
#include "graphUtils.h"
#include "dataGen.h"
#include "parallel.h"
#include <sys/types.h>
#include <sys/stat.h>
using namespace benchIO;
using namespace dataGen;
using namespace std;

typedef uint32_t numT;
typedef uint64_t idxT;

size_t filesize(const char *fname)
{ struct stat fs;
  stat(fname, &fs);
  return fs.st_size;
}

graph<intT> inputFiles(std::string prefix) {
  intT numSets, numElts;
  std::string configFilename = prefix + ".config";
  std::ifstream inf;
  inf.open (configFilename.c_str());
  inf >> numSets >> numElts;
  inf.close();

  std::string idxFilename = prefix + ".idx";
  std::string adjFilename = prefix + ".adj";
  intT adjSize = filesize(adjFilename.c_str())/sizeof(numT);

  FILE *fadj, *fidx;
  if (!(fadj = fopen(adjFilename.c_str(),"rb"))) {
    fprintf(stderr, "error: can't open %s\n", adjFilename.c_str());
    exit(1);
  }

  if (!(fidx = fopen(idxFilename.c_str(),"rb"))) {
    fprintf(stderr, "error: can't open %s\n", idxFilename.c_str());
    exit(1);
  }

  numT *tadj = newA(numT, adjSize);
  long nr = fread(tadj, sizeof(numT), adjSize,  fadj);
  if (sizeof(numT) != sizeof(intT)) {
    cout << "Wrong word size in .adj file" << endl;
    abort();
  }

  idxT *idx =  newA(idxT, 1+numSets);
  nr = fread(idx, sizeof(long), 1+numSets, fidx);

  vertex<intT> *vertices =  newA(vertex<intT>, numSets);
  parallel_for(int i=0; i < numSets; i++)
    vertices[i] = vertex<intT>((intT*) &tadj[idx[i]],idx[i+1]-idx[i]);
  free(idx);
  return graph<intT>(vertices, numSets, adjSize, (intT*) tadj);
}

int parallel_main(int argc, char* argv[]) {
  commandLine P(argc,argv,"[-d {2,3}] [-j] [-o] n <outFile>");
  char* fname = P.getArgument(0);
  graph<intT> G = inputFiles((std::string) fname);
  int r = writeGraphToFile<intT>(G, fname);
  return r;
}
