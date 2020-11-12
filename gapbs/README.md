GAP Benchmark Suite [![Build Status](https://travis-ci.org/sbeamer/gapbs.svg)](https://travis-ci.org/sbeamer/gapbs)
===================

This is the reference implementation for the [GAP](http://gap.cs.berkeley.edu/) [Benchmark Suite](http://gap.cs.berkeley.edu/benchmark.html). It is designed to be a portable high-performance baseline that only requires a compiler with support for C++11. It uses OpenMP for parallelism, but it can be compiled without OpenMP to run serially. The details of the benchmark can be found in the [specification](http://arxiv.org/abs/1508.03619).

The GAP Benchmark Suite is intended to help graph processing research by standardizing evaluations. Fewer differences between graph processing evaluations will make it easier to compare different research efforts and quantify improvements. The benchmark not only specifies graph kernels, input graphs, and evaluation methodologies, but it also provides an optimized baseline implementation (this repo). These baseline implementations are representative of state-of-the-art performance, and thus new contributions should outperform them to demonstrate an improvement.

Kernels Included
----------------
+ Breadth-First Search (BFS) - direction optimizing
+ Single-Source Shortest Paths (SSSP) - delta stepping
+ PageRank (PR) - iterative method in pull direction
+ Connected Components (CC) - Afforest & Shiloach-Vishkin
+ Betweenness Centrality (BC) - Brandes
+ Triangle Counting (TC) - Order invariant with possible relabelling


Quick Start
-----------

Build the project:

    $ make

Override the default C++ compiler:

    $ CXX=g++-8 make

Test the build:

    $ make test

Run BFS on 1,024 vertices for 1 iteration:

    $ ./bfs -g 10 -n 1

Additional command line flags can be found with `-h`


Graph Loading
-------------

All of the binaries use the same command-line options for loading graphs:
+ `-g 20` generates a Kronecker graph with 2^20 vertices (Graph500 specifications)
+ `-u 20` generates a uniform random graph with 2^20 vertices (degree 16)
+ `-f graph.el` loads graph from file graph.el
+ `-sf graph.el` symmetrizes graph loaded from file graph.el

The graph loading infrastructure understands the following formats:
+ `.el` plain-text edge-list with an edge per line as _node1_ _node2_
+ `.wel` plain-text weighted edge-list with an edge per line as _node1_ _node2_ _weight_
+ `.gr` [9th DIMACS Implementation Challenge](http://www.dis.uniroma1.it/challenge9/download.shtml) format
+ `.graph` Metis format (used in [10th DIMACS Implementation Challenge](http://www.cc.gatech.edu/dimacs10/index.shtml))
+ `.mtx` [Matrix Market](http://math.nist.gov/MatrixMarket/formats.html) format
+ `.sg` serialized pre-built graph (use `converter` to make)
+ `.wsg` weighted serialized pre-built graph (use `converter` to make)


Executing the Benchmark
-----------------------

We provide a simple makefile-based approach to automate executing the benchmark which includes fetching and building the input graphs. Using these makefiles is not a requirement of the benchmark, but we provide them as a starting point. For example, a user could save disk space by storing the input graphs in fewer formats at the expense of longer loading and conversion times. Anything that complies with the rules in the [specification](http://arxiv.org/abs/1508.03619) is allowed by the benchmark.

__*Warning:*__ A full run of this benchmark can be demanding and should probably not be done on a laptop. Building the input graphs requires about 275 GB of disk space and 64 GB of RAM. Depending on your filesystem and internet bandwidth, building the graphs can take up to 8 hours. Once the input graphs are built, you can delete `gapbs/benchmark/graphs/raw` to free up some disk space. Executing the benchmark itself will require only a few hours.

Build the input graphs:
    
    $ make bench-graphs

Execute the benchmark suite:

    $ make bench-run

Spack
-----
The GAP Benchmark Suite is also included in the [Spack](https://spack.io) package manager. To install:

    $ spack install gapbs


How to Cite
-----------

Please cite this code by the benchmark specification:

Scott Beamer, Krste Asanović, David Patterson. [*The GAP Benchmark Suite*](http://arxiv.org/abs/1508.03619). arXiv:1508.03619 [cs.DC], 2015.
