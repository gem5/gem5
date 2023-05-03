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

#ifndef A_SEQUENCE_INCLUDED
#define A_SEQUENCE_INCLUDED

#include <iostream>
#include "parallel.h"
#include "utils.h"
using namespace std;

#define _BSIZE 2048
#define _SCAN_LOG_BSIZE 10
#define _SCAN_BSIZE (1 << _SCAN_LOG_BSIZE)

template <class T>
struct _seq {
  T* A;
  long n;
  _seq() {A = NULL; n=0;}
_seq(T* _A, long _n) : A(_A), n(_n) {}
  void del() {free(A);}
};

template <class E>
void brokenCompiler__(intT n, E* x, E v) {
  parallel_for(intT i=0; i<n; i++) x[i] = v;
}

template <class E>
static E* newArray(intT n, E v) {
  E* x = (E*) malloc(n*sizeof(E));
  brokenCompiler__(n, x, v);
  return x;
}

namespace sequence {

  template <class intT>
  struct boolGetA {
    bool* A;
    boolGetA(bool* AA) : A(AA) {}
    intT operator() (intT i) {return (intT) A[i];}
  };

  template <class ET, class intT>
  struct getA {
    ET* A;
    getA(ET* AA) : A(AA) {}
    ET operator() (intT i) {return A[i];}
  };

  template <class IT, class OT, class intT, class F>
  struct getAF {
    IT* A;
    F f;
    getAF(IT* AA, F ff) : A(AA), f(ff) {}
    OT operator () (intT i) {return f(A[i]);}
  };

#define nblocks(_n,_bsize) (1 + ((_n)-1)/(_bsize))

#define blocked_for(_i, _s, _e, _bsize, _body)  {	\
    intT _ss = _s;					\
    intT _ee = _e;					\
    intT _n = _ee-_ss;					\
    intT _l = nblocks(_n,_bsize);			\
    parallel_for (intT _i = 0; _i < _l; _i++) {		\
      intT _s = _ss + _i * (_bsize);			\
      intT _e = min(_s + (_bsize), _ee);			\
      _body						\
	}						\
  }

  template <class OT, class intT, class F, class G>
  OT reduceSerial(intT s, intT e, F f, G g) {
    OT r = g(s);
    for (intT j=s+1; j < e; j++) r = f(r,g(j));
    return r;
  }

  template <class OT, class intT, class F, class G>
  OT reduce(intT s, intT e, F f, G g) {
    intT l = nblocks(e-s, _SCAN_BSIZE);
    if (l <= 1) return reduceSerial<OT>(s, e, f , g);
    OT *Sums = newA(OT,l);
    blocked_for (i, s, e, _SCAN_BSIZE,
		 Sums[i] = reduceSerial<OT>(s, e, f, g););
    OT r = reduce<OT>((intT) 0, l, f, getA<OT,intT>(Sums));
    free(Sums);
    return r;
  }

  template <class OT, class intT, class F>
  OT reduce(OT* A, intT n, F f) {
    return reduce<OT>((intT)0,n,f,getA<OT,intT>(A));
  }

  template <class OT, class intT>
  OT plusReduce(OT* A, intT n) {
    return reduce<OT>((intT)0,n,utils::addF<OT>(),getA<OT,intT>(A));
  }

  // g is the map function (applied to each element)
  // f is the reduce function
  // need to specify OT since it is not an argument
  template <class OT, class IT, class intT, class F, class G>
  OT mapReduce(IT* A, intT n, F f, G g) {
    return reduce<OT>((intT) 0,n,f,getAF<IT,OT,intT,G>(A,g));
  }

  template <class ET, class intT, class F, class G>
  intT maxIndexSerial(intT s, intT e, F f, G g) {
    ET r = g(s);
    intT k = 0;
    for (intT j=s+1; j < e; j++) {
      ET v = g(j);
      if (f(v,r)) { r = v; k = j;}
    }
    return k;
  }

  template <class ET, class intT, class F, class G>
  intT maxIndex(intT s, intT e, F f, G g) {
    intT l = nblocks(e-s, _SCAN_BSIZE);
    if (l <= 2) return maxIndexSerial<ET>(s, e, f , g);
    else {
      intT *Idx = newA(intT,l);
      blocked_for (i, s, e, _SCAN_BSIZE,
		   Idx[i] = maxIndexSerial<ET>(s, e, f, g););
      intT k = Idx[0];
      for (intT j=1; j < l; j++)
	if (f(g(Idx[j]),g(k))) k = Idx[j];
      free(Idx);
      return k;
    }
  }

  template <class ET, class intT, class F>
  intT maxIndex(ET* A, intT n, F f) {
    return maxIndex<ET>((intT) 0, n, f, getA<ET,intT>(A));
  }

  template <class ET, class intT, class F, class G>
  ET scanSerial(ET* Out, intT s, intT e, F f, G g, ET zero, bool inclusive, bool back) {
    ET r = zero;

    if (inclusive) {
      if (back) for (intT i = e-1; i >= s; i--) Out[i] = r = f(r,g(i));
      else for (intT i = s; i < e; i++) Out[i] = r = f(r,g(i));
    } else {
      if (back)
	for (intT i = e-1; i >= s; i--) {
	  ET t = g(i);
	  Out[i] = r;
	  r = f(r,t);
	}
      else
	for (intT i = s; i < e; i++) {
	  ET t = g(i);
	  Out[i] = r;
	  r = f(r,t);
	}
    }
    return r;
  }

  template <class ET, class intT, class F>
  ET scanSerial(ET *In, ET* Out, intT n, F f, ET zero) {
    return scanSerial(Out, (intT) 0, n, f, getA<ET,intT>(In), zero, false, false);
  }

  // back indicates it runs in reverse direction
  template <class ET, class intT, class F, class G>
  ET scan(ET* Out, intT s, intT e, F f, G g,  ET zero, bool inclusive, bool back) {
    intT n = e-s;
    intT l = nblocks(n,_SCAN_BSIZE);
    if (l <= 2) return scanSerial(Out, s, e, f, g, zero, inclusive, back);
    ET *Sums = newA(ET,nblocks(n,_SCAN_BSIZE));
    blocked_for (i, s, e, _SCAN_BSIZE,
		 Sums[i] = reduceSerial<ET>(s, e, f, g););
    ET total = scan(Sums, (intT) 0, l, f, getA<ET,intT>(Sums), zero, false, back);
    blocked_for (i, s, e, _SCAN_BSIZE,
		 scanSerial(Out, s, e, f, g, Sums[i], inclusive, back););
    free(Sums);
    return total;
  }

  template <class ET, class intT, class F>
  ET scan(ET *In, ET* Out, intT n, F f, ET zero) {
    return scan(Out, (intT) 0, n, f, getA<ET,intT>(In), zero, false, false);}

  template <class ET, class intT, class F>
  ET scanBack(ET *In, ET* Out, intT n, F f, ET zero) {
    return scan(Out, (intT) 0, n, f, getA<ET,intT>(In), zero, false, true);}

  template <class ET, class intT, class F>
  ET scanI(ET *In, ET* Out, intT n, F f, ET zero) {
    return scan(Out, (intT) 0, n, f, getA<ET,intT>(In), zero, true, false);}

  template <class ET, class intT, class F>
  ET scanIBack(ET *In, ET* Out, intT n, F f, ET zero) {
    return scan(Out, (intT) 0, n, f, getA<ET,intT>(In), zero, true, true);}

  template <class ET, class intT>
  ET plusScan(ET *In, ET* Out, intT n) {
    return scan(Out, (intT) 0, n, utils::addF<ET>(), getA<ET,intT>(In),
		(ET) 0, false, false);}

  template <class intT>
  intT enumerate(bool *In, intT* Out, intT n) {
    return scan(Out, (intT) 0, n, utils::addF<intT>(), boolGetA<intT>(In),
		(intT) 0, false, false);}


#define _F_BSIZE (2*_SCAN_BSIZE)

  // sums a sequence of n boolean flags
  // an optimized version that sums blocks of 4 booleans by treating
  // them as an integer
  // Only optimized when n is a multiple of 512 and Fl is 4byte aligned
  template <class intT>
  intT sumFlagsSerial(bool *Fl, intT n) {
    intT r = 0;
    if (n >= 128 && (n & 511) == 0 && ((long) Fl & 3) == 0) {
      int* IFl = (int*) Fl;
      for (int k = 0; k < (n >> 9); k++) {
	int rr = 0;
	for (int j=0; j < 128; j++) rr += IFl[j];
	r += (rr&255) + ((rr>>8)&255) + ((rr>>16)&255) + ((rr>>24)&255);
	IFl += 128;
      }
    } else for (intT j=0; j < n; j++) r += Fl[j];
    return r;
  }

  template <class ET, class intT, class F>
  _seq<ET> packSerial(ET* Out, bool* Fl, intT s, intT e, F f) {
    if (Out == NULL) {
      intT m = sumFlagsSerial(Fl+s, e-s);
      Out = newA(ET,m);
    }
    intT k = 0;
    for (intT i=s; i < e; i++) if (Fl[i]) Out[k++] = f(i);
    return _seq<ET>(Out,k);
  }

  template <class ET, class intT, class F>
  _seq<ET> pack(ET* Out, bool* Fl, intT s, intT e, F f) {
    intT l = nblocks(e-s, _F_BSIZE);
    if (l <= 1) return packSerial(Out, Fl, s, e, f);
    intT *Sums = newA(intT,l);
    blocked_for (i, s, e, _F_BSIZE, Sums[i] = sumFlagsSerial(Fl+s, e-s););
    intT m = plusScan(Sums, Sums, l);
    if (Out == NULL) Out = newA(ET,m);
    blocked_for(i, s, e, _F_BSIZE, packSerial(Out+Sums[i], Fl, s, e, f););
    free(Sums);
    return _seq<ET>(Out,m);
  }

  template <class ET, class intT, class F>
  pair<_seq<ET>,_seq<ET> > pack2(ET* Out, bool* Fl1, bool* Fl2,
				 intT s, intT e, F f) {
    intT l = nblocks(e-s, _F_BSIZE);
    intT *Sums1 = newA(intT,l);
    intT *Sums2 = newA(intT,l);
    blocked_for (i, s, e, _F_BSIZE,
                 Sums1[i] = sumFlagsSerial(Fl1+s, e-s);
                 Sums2[i] = sumFlagsSerial(Fl2+s, e-s););
    intT m1 = plusScan(Sums1, Sums1, l);
    intT m2 = plusScan(Sums2, Sums2, l);
    ET* Out1;
    ET* Out2;
    if (Out == NULL) {
      Out1 = newA(ET,m1);
      Out2 = newA(ET,m2);
    } else {
      Out1 = Out;
      Out2 = Out+m1;
    }
    blocked_for(i, s, e, _F_BSIZE,
		packSerial(Out1+Sums1[i], Fl1, s, e, f);
		packSerial(Out2+Sums2[i], Fl2, s, e, f););
    free(Sums1); free(Sums2);
    return pair<_seq<ET>,_seq<ET> >(_seq<ET>(Out1,m1),_seq<ET>(Out2,m2));
  }

  template <class ET, class intT>
  intT pack(ET* In, ET* Out, bool* Fl, intT n) {
    return pack(Out, Fl, (intT) 0, n, getA<ET,intT>(In)).n;}

  template <class ET, class intT>
  pair<intT,intT> pack2(ET* In, ET* Out, bool* Fl1, bool* Fl2, intT n) {
    pair<_seq<ET>,_seq<ET> > r;
    r = pack2(Out, Fl1, Fl2, (intT) 0, n, getA<ET,intT>(In));
    return pair<intT,intT>(r.first.n,r.second.n);
  }

  template <class ET, class intT>
  _seq<ET> pack(ET* In, bool* Fl, intT n) {
    return pack((ET*) NULL, Fl, (intT) 0, n, getA<ET,intT>(In));}

  template <class intT>
  intT packIndex(intT* Out, bool* Fl, intT n) {
    return pack(Out, Fl, (intT) 0, n, utils::identityF<intT>()).n;
  }

  template <class intT>
  _seq<intT> packIndex(bool* Fl, intT n) {
    return pack((intT *) NULL, Fl, (intT) 0, n, utils::identityF<intT>());
  }

  template <class ET, class intT, class PRED>
  intT filter(ET* In, ET* Out, intT n, PRED p) {
    bool *Fl = newA(bool,n);
    parallel_for (intT i=0; i < n; i++) Fl[i] = (bool) p(In[i]);
    intT  m = pack(In, Out, Fl, n);
    free(Fl);
    return m;
  }

  template <class ET, class intT, class PRED>
  _seq<ET> filter(ET* In, intT n, PRED p) {
    bool *Fl = newA(bool,n);
    parallel_for (intT i=0; i < n; i++) Fl[i] = (bool) p(In[i]);
    _seq<ET> R = pack(In, Fl, n);
    free(Fl);
    return R;
  }

}

#endif // _A_SEQUENCE_INCLUDED
