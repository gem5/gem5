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



int parallel_main(int argc, char* argv[]) {
  commandLine P(argc,argv,"-o <outFile>");
  char* iFile = P.getArgument(0);
  char* oFile = P.getOptionValue("-o");

  graph<intT> G = readGraphFromFile<intT>(iFile);
  graphCheckConsistency(G);

  edgeArray<intT> EA = edgesFromGraph(G);
  G.del();
  // intT* A = newA(intT,EA.nonZeros);
  // parallel_for(int i=0;i<EA.nonZeros;i++){
  //   if(EA.E[i].u == EA.E[i].v) A[i] = 0;
  //   else A[i] = 1;
  // }

  // intT m = sequence::plusScan(A,A,EA.nonZeros);
  // cout << m << " " << EA.nonZeros << " " << EA.numRows << endl;

  // if(m < EA.nonZeros){
  //   edge<intT>* E_packed = newA(edge<intT>,m);
  //   E_packed[0] = EA.E[0];
  //   parallel_for(int i=1;i<EA.nonZeros;i++){
  //     if(A[i] != A[i]-1) E_packed[A[i]] = EA.E[i];
  //   }
  //   EA.nonZeros = m;
  //   free(EA.E);
  //   EA.E = E_packed;
  // }
  // free(A);
  G = graphFromEdges(EA,0);
  //EA.del();
  writeGraphToFile(G,oFile);
  G.del();
}
