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

#ifndef A_RADIX_INCLUDED
#define A_RADIX_INCLUDED

#include <iostream>
#include <math.h>
#include "parallel.h"
#include "sequence.h"
#include "utils.h"
#include "transpose.h"
using namespace std;

namespace intSort {

  // Cannot be greater than 8 without changing definition of bIndexT
  //    from unsigned char to unsigned int (or unsigned short)
#define MAX_RADIX 8
#define BUCKETS 256    // 1 << MAX_RADIX

  //typedef int bucketsT[BUCKETS];


  // a type that must hold MAX_RADIX bits
  typedef unsigned char bIndexT;

  template <class E, class F, class intT>
  void radixBlock(E* A, E* B, bIndexT *Tmp, intT counts[BUCKETS], intT offsets[BUCKETS],
		  intT Boffset, intT n, intT m, F extract) {

    for (intT i = 0; i < m; i++)  counts[i] = 0;
    for (intT j = 0; j < n; j++) {
      intT k = Tmp[j] = extract(A[j]);
      counts[k]++;
    }
    intT s = Boffset;
    for (intT i = 0; i < m; i++) {
      s += counts[i];
      offsets[i] = s;
    }
    for (intT j = n-1; j >= 0; j--) {
      intT x =  --offsets[Tmp[j]];
      B[x] = A[j];
    }
  }

  template <class E, class F, class intT>
  void radixStepSerial(E* A, E* B, bIndexT *Tmp, intT buckets[BUCKETS],
		       intT n, intT m, F extract) {
    radixBlock(A, B, Tmp, buckets, buckets, (intT)0, n, m, extract);
    for (intT i=0; i < n; i++) A[i] = B[i];
    return;
  }

  // A is the input and sorted output (length = n)
  // B is temporary space for copying data (length = n)
  // Tmp is temporary space for extracting the bytes (length = n)
  // BK is an array of bucket sets, each set has BUCKETS integers
  //    it is used for temporary space for bucket counts and offsets
  // numBK is the length of BK (number of sets of buckets)
  // the first entry of BK is also used to return the offset of each bucket
  // m is the number of buckets per set (m <= BUCKETS)
  // extract is a function that extract the appropriate bits from A
  //  it must return a non-negative integer less than m
  template <class E, class F, class intT>
    void radixStep(E* A, E* B, bIndexT *Tmp, intT (*BK)[BUCKETS],
		 intT numBK, intT n, intT m, bool top, F extract) {


    // need 3 bucket sets per block
    int expand = (sizeof(E)<=4) ? 64 : 32;
    intT blocks = min(numBK/3,(1+n/(BUCKETS*expand)));

    if (blocks < 2) {
      radixStepSerial(A, B, Tmp, BK[0], n, m, extract);
      return;
    }
    intT nn = (n+blocks-1)/blocks;
    intT* cnts = (intT*) BK;
    intT* oA = (intT*) (BK+blocks);
    intT* oB = (intT*) (BK+2*blocks);

    parallel_for_1 (intT i=0; i < blocks; i++) {
      intT od = i*nn;
      intT nni = min(max<intT>(n-od,0),nn);
      radixBlock(A+od, B, Tmp+od, cnts + m*i, oB + m*i, od, nni, m, extract);
    }

    transpose<intT,intT>(cnts, oA).trans(blocks, m);

    intT ss;
    if (top)
      ss = sequence::scan(oA, oA, blocks*m, utils::addF<intT>(),(intT)0);
    else
      ss = sequence::scanSerial(oA, oA, blocks*m, utils::addF<intT>(),(intT)0);
    //utils::myAssert(ss == n, "radixStep: sizes don't match");

    blockTrans<E,intT>(B, A, oB, oA, cnts).trans(blocks, m);

    // put the offsets for each bucket in the first bucket set of BK
    for (intT j = 0; j < m; j++) BK[0][j] = oA[j*blocks];
  }

  // a function to extract "bits" bits starting at bit location "offset"
  template <class E, class F>
    struct eBits {
      F _f;  intT _mask;  intT _offset;
      eBits(int bits, intT offset, F f): _mask((1<<bits)-1),
					_offset(offset), _f(f) {}
      intT operator() (E p) {return _mask&(_f(p)>>_offset);}
    };

  // Radix sort with low order bits first
  template <class E, class F, class intT>
    void radixLoopBottomUp(E *A, E *B, bIndexT *Tmp, intT (*BK)[BUCKETS],
			 intT numBK, intT n, int bits, bool top, F f) {
      int rounds = 1+(bits-1)/MAX_RADIX;
      int rbits = 1+(bits-1)/rounds;
      int bitOffset = 0;
      while (bitOffset < bits) {
	if (bitOffset+rbits > bits) rbits = bits-bitOffset;
	radixStep(A, B, Tmp, BK, numBK, n, (intT)1 << rbits, top,
		  eBits<E,F>(rbits,bitOffset,f));
	bitOffset += rbits;
      }
  }

  // Radix sort with high order bits first
  template <class E, class F, class intT>
    void radixLoopTopDown(E *A, E *B, bIndexT *Tmp, intT (*BK)[BUCKETS],
			intT numBK, intT n, int bits, F f) {
    if (n == 0) return;
    if (bits <= MAX_RADIX) {
      radixStep(A, B, Tmp, BK, numBK, n, (intT)1 << bits, true, eBits<E,F>(bits,0,f));
    } else if (numBK >= BUCKETS+1) {
      radixStep(A, B, Tmp, BK, numBK, n, (intT)BUCKETS, true,
		eBits<E,F>(MAX_RADIX,bits-MAX_RADIX,f));
      intT* offsets = BK[0];
      intT remain = numBK - BUCKETS - 1;
      float y = remain / (float) n;
      parallel_for (int i=0; i < BUCKETS; i++) {
	intT segOffset = offsets[i];
	intT segNextOffset = (i == BUCKETS-1) ? n : offsets[i+1];
	intT segLen = segNextOffset - segOffset;
	intT blocksOffset = ((intT) floor(segOffset * y)) + i + 1;
	intT blocksNextOffset = ((intT) floor(segNextOffset * y)) + i + 2;
	intT blockLen = blocksNextOffset - blocksOffset;
	radixLoopTopDown(A + segOffset, B + segOffset, Tmp + segOffset,
			 BK + blocksOffset, blockLen, segLen,
			 bits-MAX_RADIX, f);
      }
    } else {
      radixLoopBottomUp(A, B, Tmp, BK, numBK, n, bits, false, f);
    }
  }

  template <class E, class intT>
  long iSortSpace(intT n) {
    typedef intT bucketsT[BUCKETS];
    intT numBK = 1+n/(BUCKETS*8);
    return sizeof(E)*n + sizeof(bIndexT)*n + sizeof(bucketsT)*numBK;
  }

  // Sorts the array A, which is of length n.
  // Function f maps each element into an integer in the range [0,m)
  // If bucketOffsets is not NULL then it should be an array of length m
  // The offset in A of each bucket i in [0,m) is placed in location i
  //   such that for i < m-1, offsets[i+1]-offsets[i] gives the number
  //   of keys=i.   For i = m-1, n-offsets[i] is the number.
  template <class E, class F, class intT>
  void iSort(E *A, intT* bucketOffsets, intT n, intT m, bool bottomUp,
	     char* tmpSpace, F f) {

    typedef intT bucketsT[BUCKETS];


    int bits = utils::log2Up(m);
    intT numBK = 1+n/(BUCKETS*8);

    // the temporary space is broken into 3 parts: B, Tmp and BK
    E *B = (E*) tmpSpace;
    intT Bsize =sizeof(E)*n;
    bIndexT *Tmp = (bIndexT*) (tmpSpace+Bsize); // one byte per item
    intT tmpSize = sizeof(bIndexT)*n;
    bucketsT *BK = (bucketsT*) (tmpSpace+Bsize+tmpSize);
    if (bits <= MAX_RADIX) {
      radixStep(A, B, Tmp, BK, numBK, n, (intT) 1 << bits, true, eBits<E,F>(bits,0,f));
      if (bucketOffsets != NULL) {
	parallel_for (intT i=0; i < m; i++)
	  bucketOffsets[i] = BK[0][i];
      }
      return;
    } else if (bottomUp)
      radixLoopBottomUp(A, B, Tmp, BK, numBK, n, bits, true, f);
    else
      radixLoopTopDown(A, B, Tmp, BK, numBK, n, bits, f);
    if (bucketOffsets != NULL) {
      {parallel_for (intT i=0; i < m; i++) bucketOffsets[i] = n;}
      {parallel_for (intT i=0; i < n-1; i++) {
	  intT v = f(A[i]);
	  intT vn = f(A[i+1]);
	  if (v != vn) bucketOffsets[vn] = i+1;
	}}
      bucketOffsets[f(A[0])] = 0;
      sequence::scanIBack(bucketOffsets, bucketOffsets, (intT) m,
			  utils::minF<intT>(), (intT) n);
    }
  }

  template <class E, class F, class intT>
  void iSort(E *A, intT* bucketOffsets, intT n, intT m, bool bottomUp, F f) {
    long x = iSortSpace<E,intT>(n);
    char* s = (char*) malloc(x);
    iSort(A, bucketOffsets, n, m, bottomUp, s, f);
    free(s);
  }

  template <class E, class F, class intT>
  void iSort(E *A, intT* bucketOffsets, intT n, intT m, F f) {
    iSort(A, bucketOffsets, n, m, false, f);}

  // A version that uses a NULL bucketOffset
  template <class E, class Func, class intT>
  void iSort(E *A, intT n, intT m, Func f) {
    iSort(A, (intT*) NULL, n, m, false, f);}

  template <class E, class F, class intT>
  void iSortBottomUp(E *A, intT n, intT m, F f) {
    iSort(A, (intT*) NULL, n, m, true, f);}
};

typedef unsigned int uint;

template <class intT>
static void integerSort(uintT *A, intT n) {
  intT maxV = sequence::reduce(A,n,utils::maxF<uintT>());
  intSort::iSort(A, (intT*) NULL, n, maxV+1,  utils::identityF<uintT>());
}

template <class intT>
static void integerSort(uintT *A, intT n, char* s) {
  intT maxV = sequence::reduce(A,n,utils::maxF<uintT>());
  intSort::iSort(A, (intT*) NULL, n, maxV+1, false, s, utils::identityF<uintT>());
}

template <class T, class intT>
void integerSort(pair<uintT,T> *A, intT n) {
  intT maxV = sequence::mapReduce<uintT>(A,n,utils::maxF<uintT>(),
					utils::firstF<uintT,T>());
  intSort::iSort(A, (intT*) NULL, n, maxV+1,  utils::firstF<uintT,T>());
}

template <class T, class intT>
void integerSort(pair<uintT,T> *A, intT n, char* s) {
  intT maxV = sequence::mapReduce<uintT>(A,n,utils::maxF<uintT>(),
					utils::firstF<uintT,T>());
  intSort::iSort(A, (intT*) NULL, n, maxV+1, false, s, utils::firstF<uintT,T>());
}

#endif
