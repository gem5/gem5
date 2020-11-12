// Copyright (c) 2015, The Regents of the University of California (Regents)
// See LICENSE.txt for license details

#include <algorithm>
#include <iostream>
#include <vector>

#include "benchmark.h"
#include "builder.h"
#include "command_line.h"
#include "graph.h"
#include "pvector.h"


/*
GAP Benchmark Suite
Kernel: PageRank (PR)
Author: Scott Beamer

Will return pagerank scores for all vertices once total change < epsilon

This PR implementation uses the traditional iterative approach. This is done
to ease comparisons to other implementations (often use same algorithm), but
it is not necesarily the fastest way to implement it. It does perform the
updates in the pull direction to remove the need for atomics.
*/


using namespace std;

typedef float ScoreT;
const float kDamp = 0.85;

pvector<ScoreT> PageRankPull(const Graph &g, int max_iters,
                             double epsilon = 0) {
  const ScoreT init_score = 1.0f / g.num_nodes();
  const ScoreT base_score = (1.0f - kDamp) / g.num_nodes();
  pvector<ScoreT> scores(g.num_nodes(), init_score);
  pvector<ScoreT> outgoing_contrib(g.num_nodes());
  for (int iter=0; iter < max_iters; iter++) {
    double error = 0;
    #pragma omp parallel for
    for (NodeID n=0; n < g.num_nodes(); n++)
      outgoing_contrib[n] = scores[n] / g.out_degree(n);
    #pragma omp parallel for reduction(+ : error) schedule(dynamic, 64)
    for (NodeID u=0; u < g.num_nodes(); u++) {
      ScoreT incoming_total = 0;
      for (NodeID v : g.in_neigh(u))
        incoming_total += outgoing_contrib[v];
      ScoreT old_score = scores[u];
      scores[u] = base_score + kDamp * incoming_total;
      error += fabs(scores[u] - old_score);
    }
    printf(" %2d    %lf\n", iter, error);
    if (error < epsilon)
      break;
  }
  return scores;
}


void PrintTopScores(const Graph &g, const pvector<ScoreT> &scores) {
  vector<pair<NodeID, ScoreT>> score_pairs(g.num_nodes());
  for (NodeID n=0; n < g.num_nodes(); n++) {
    score_pairs[n] = make_pair(n, scores[n]);
  }
  int k = 5;
  vector<pair<ScoreT, NodeID>> top_k = TopK(score_pairs, k);
  k = min(k, static_cast<int>(top_k.size()));
  for (auto kvp : top_k)
    cout << kvp.second << ":" << kvp.first << endl;
}


// Verifies by asserting a single serial iteration in push direction has
//   error < target_error
bool PRVerifier(const Graph &g, const pvector<ScoreT> &scores,
                        double target_error) {
  const ScoreT base_score = (1.0f - kDamp) / g.num_nodes();
  pvector<ScoreT> incomming_sums(g.num_nodes(), 0);
  double error = 0;
  for (NodeID u : g.vertices()) {
    ScoreT outgoing_contrib = scores[u] / g.out_degree(u);
    for (NodeID v : g.out_neigh(u))
      incomming_sums[v] += outgoing_contrib;
  }
  for (NodeID n : g.vertices()) {
    error += fabs(base_score + kDamp * incomming_sums[n] - scores[n]);
    incomming_sums[n] = 0;
  }
  PrintTime("Total Error", error);
  return error < target_error;
}


int main(int argc, char* argv[]) {
  CLPageRank cli(argc, argv, "pagerank", 1e-4, 20);
  if (!cli.ParseArgs())
    return -1;
  Builder b(cli);
  Graph g = b.MakeGraph();
  auto PRBound = [&cli] (const Graph &g) {
    return PageRankPull(g, cli.max_iters(), cli.tolerance());
  };
  auto VerifierBound = [&cli] (const Graph &g, const pvector<ScoreT> &scores) {
    return PRVerifier(g, scores, cli.tolerance());
  };
  BenchmarkKernel(cli, g, PRBound, PrintTopScores, VerifierBound);
  return 0;
}
