This is an archive of benchmarks and a script for running gem5.

The archive comes pre-built for x86-64 Linux. On a Unix-like
system with GNU make installed, you can also use 'make clean',
followed by 'make' to rebuild them. You may need to adjust the
supplied Makefile, and the Makefiles in some subdirectories,
if you want to use a compiler other than GCC.

Included in this directory is a script for running gem5 called
gem5script.py. This is intended to be modified to complete the
assignment. It should be reasonably well commented.

The included benchmarks are also described in the program writeup:

*  queens --- solves the N queens problem. Source and license is in
   queens.c. I obtained this from the LLVM's
   [test-suite repository](https://github.com/llvm-mirror/test-suite),
   and the program was authored by Roberto Sierra.

*  BFS --- performs a breadth-first search. Source and license and
   utilities for generating input graphs are in the minSpanningForest
   directory. I obtained this from the
   [Problem Based Benchmark Suite](http://www.cs.cmu.edu/~pbbs/benchmarks/breadthFirstSearch.html),

*  blocked-matmul --- does a 84x84 register-blocked matrix multiply.
   Source is in blocked-matmul.c.

*  sha --- an implementation of NIST's SHA (Secure Hash Algorithm).
   I obtained this from [MIBench](http://vhosts.eecs.umich.edu/mibench/).
   Source code is in sha-src. According to comments in the source, this
   code was written by Peter C. Gutmann and Uwe Hollerbach.

The `inputs` directory contains example input files for the BFS and
sha benchmarks.
