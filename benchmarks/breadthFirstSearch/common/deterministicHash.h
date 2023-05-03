// This code is part of the Problem Based Benchmark Suite (PBBS)
// Copyright (c) 2010 Guy Blelloch and the PBBS team
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

#ifndef A_HASH_INCLUDED
#define A_HASH_INCLUDED

#include "parallel.h"
#include "utils.h"
#include "sequence.h"
using namespace std;

// A "history independent" hash table that supports insertion, and searching
// It is described in the paper
//   Guy E. Blelloch, Daniel Golovin
//   Strongly History-Independent Hashing with Applications
//   FOCS 2007: 272-282
// At any quiescent point (when no operations are actively updating the
//   structure) the state will depend only on the keys it contains and not
//   on the history of the insertion order.
// Insertions can happen in parallel, but they cannot overlap with searches
// Searches can happen in parallel
// Deletions must happen sequentially
template <class HASH, class intT>
class Table {
 private:
  typedef typename HASH::eType eType;
  typedef typename HASH::kType kType;
  intT m;
  intT mask;
  eType empty;
  HASH hashStruct;
  eType* TA;
  intT* compactL;

  // needs to be in separate routine due to Cilk bugs
  static void clearA(eType* A, intT n, eType v) {
    parallel_for (intT i=0; i < n; i++) A[i] = v;
  }

  struct notEmptyF {
    eType e; notEmptyF(eType _e) : e(_e) {}
    int operator() (eType a) {return e != a;}};

  uintT hashToRange(intT h) {return h & mask;}
  intT firstIndex(kType v) {return hashToRange(hashStruct.hash(v));}
  intT incrementIndex(intT h) {return hashToRange(h+1);}
  intT decrementIndex(intT h) {return hashToRange(h-1);}
  bool lessIndex(intT a, intT b) {return 2 * hashToRange(a - b) > m;}


 public:
  // Size is the maximum number of values the hash table will hold.
  // Overfilling the table could put it into an infinite loop.
  Table(intT size, HASH hashF) :
    m(1 << utils::log2Up(100+2*size)),
    mask(m-1),
    empty(hashF.empty()),
    hashStruct(hashF),
    TA(newA(eType,m)),
    compactL(NULL)
      { clearA(TA,m,empty); }

  // Deletes the allocated arrays
  void del() {
    free(TA);
    if (compactL != NULL) free(compactL);
  }

  // prioritized linear probing
  //   a new key will bump an existing key up if it has a higher priority
  //   an equal key will replace an old key if replaceQ(new,old) is true
  // returns 0 if not inserted (i.e. equal and replaceQ false) and 1 otherwise
  bool insert(eType v) {
    kType vkey = hashStruct.getKey(v);
    intT h = firstIndex(vkey);
    while (1) {
      eType c;
      int cmp;
      bool swapped = 0;
      c = TA[h];
      cmp = (c==empty) ? 1 : hashStruct.cmp(vkey,hashStruct.getKey(c));

      // while v is higher priority than entry at TA[h] try to swap it in
      while (cmp == 1 && !(swapped=utils::CAS(&TA[h],c,v))) {
	c = TA[h];
	cmp = hashStruct.cmp(vkey,hashStruct.getKey(c));
      }

      // if swap succeeded either we are done (if swapped with empty)
      // or we have a new lower priority value we have to insert
      if (swapped) {
	if (c==empty) return 1; // done
	else { v = c; vkey = hashStruct.getKey(v);} // new value to insert

      } else {
	// if swap did not succeed then priority of TA[h] >= priority of v

        // if equal keys (priorities equal) then either quit or try to replace
	while (cmp == 0) {
	  // if other equal element does not need to be replaced then quit
	  if (!hashStruct.replaceQ(v,c)) return 0;

	  // otherwise try to replace (atomically) and quit if successful
	  else if (utils::CAS(&TA[h],c,v)) return 1;

          // otherwise failed due to concurrent write, try again
	  c = TA[h];
	  cmp = hashStruct.cmp(vkey,hashStruct.getKey(c));
	}
      }

      // move to next bucket
      h = incrementIndex(h);
    }
    return 0; // should never get here
  }

  // needs to be more thoroughly tested
  // currently always returns true
  bool deleteVal(kType v) {
    intT i = firstIndex(v);
    int cmp;

    // find first element less than or equal to v in priority order
    intT j = i;
    eType c = TA[j];
    while ((cmp = (c==empty) ? 1 : hashStruct.cmp(v,hashStruct.getKey(c))) < 0) {
      j = incrementIndex(j);
      c = TA[j];
    }
    do {
      if (cmp > 0) {
        // value at j is less than v, need to move down one
	if (j == i) return true;
	j = decrementIndex(j);
      }
      else { // (cmp == 0)
        // found the element to delete at location j

	// Find next available element to fill location j.
        // This is a little tricky since we need to skip over elements for
        // which the hash index is greater than j, and need to account for
        // things being moved around by others as we search.
        // Makes use of the fact that values in a cell can only decrease
        // during a delete phase as elements are moved from the right to left.
	intT jj = incrementIndex(j);
	eType x = TA[jj];
	while (x != empty && lessIndex(j, firstIndex(hashStruct.getKey(x)))) {
	  jj = incrementIndex(jj);
	  x = TA[jj];
	}
	intT jjj = decrementIndex(jj);
	while (jjj != j) {
	  eType y = TA[jjj];
	  if (y == empty || !lessIndex(j, firstIndex(hashStruct.getKey(y)))) x = y;
	  jjj = decrementIndex(jjj);
	}

	// try to copy the the replacement element into j
	if (utils::CAS(&TA[j],c,x)) {
          // swap was successful
          // if the replacement element was empty, we are done
	  if (x == empty) return true;

	  // Otherwise there are now two copies of the replacement element x
          // delete one copy (probably the original) by starting to look at jj.
          // Note that others can come along in the meantime and delete
          // one or both of them, but that is fine.
	  v = hashStruct.getKey(x);
	  j = jj;
	} else {
	  // if fails then c (with value v) has been deleted or moved to a lower
          // location by someone else.
          // start looking at one location lower
	  if (j == i) return true;
	  j = decrementIndex(j);
	}
      }
      c = TA[j];
      cmp = (c == empty) ? 1 : hashStruct.cmp(v, hashStruct.getKey(c));
    } while (cmp >= 0);
    return true;
  }

  // Returns the value if an equal value is found in the table
  // otherwise returns the "empty" element.
  // due to prioritization, can quit early if v is greater than cell
  eType find(kType v) {
    intT h = firstIndex(v);
    eType c = TA[h];
    while (1) {
      if (c == empty) return empty;
      int cmp = hashStruct.cmp(v,hashStruct.getKey(c));
      if (cmp >= 0)
	if (cmp == 1) return empty;
	else return c;
      h = incrementIndex(h);
      c = TA[h];
    }
  }

  // returns the number of entries
  intT count() {
    return sequence::mapReduce<intT>(TA,m,utils::addF<intT>(),notEmptyF(empty));
  }

  // returns all the current entries compacted into a sequence
  _seq<eType> entries() {
    bool *FL = newA(bool,m);
    parallel_for (intT i=0; i < m; i++)
      FL[i] = (TA[i] != empty);
    _seq<eType> R = sequence::pack(TA,FL,m);
    free(FL);
    return R;
  }

  // prints the current entries along with the index they are stored at
  void print() {
    cout << "vals = ";
    for (intT i=0; i < m; i++)
      if (TA[i] != empty)
	cout << i << ":" << TA[i] << ",";
    cout << endl;
  }
};

template <class HASH, class ET, class intT>
_seq<ET> removeDuplicates(_seq<ET> S, intT m, HASH hashF) {
  Table<HASH,intT> T(m,hashF);
  ET* A = S.A;
  {parallel_for(intT i = 0; i < S.n; i++) { T.insert(A[i]);}}
  _seq<ET> R = T.entries();
  T.del();
  return R;
}

template <class HASH, class ET>
_seq<ET> removeDuplicates(_seq<ET> S, HASH hashF) {
  return removeDuplicates(S, S.n, hashF);
}

template <class intT>
struct hashInt {
  typedef intT eType;
  typedef intT kType;
  eType empty() {return -1;}
  kType getKey(eType v) {return v;}
  intT hash(kType v) {return utils::hash(v);}
  int cmp(kType v, kType b) {return (v > b) ? 1 : ((v == b) ? 0 : -1);}
  bool replaceQ(eType v, eType b) {return 0;}
};

// works for non-negative integers (uses -1 to mark cell as empty)

static _seq<intT> removeDuplicates(_seq<intT> A) {
  return removeDuplicates(A,hashInt<intT>());
}

//typedef Table<hashInt> IntTable;
//static IntTable makeIntTable(int m) {return IntTable(m,hashInt());}
template <class intT>
static Table<hashInt<intT>,intT > makeIntTable(intT m) {
  return Table<hashInt<intT>,intT >(m,hashInt<intT>());}

struct hashStr {
  typedef char* eType;
  typedef char* kType;

  eType empty() {return NULL;}
  kType getKey(eType v) {
    return v;}

  uintT hash(kType s) {
    uintT hash = 0;
    while (*s) hash = *s++ + (hash << 6) + (hash << 16) - hash;
    return hash;
  }

  int cmp(kType s, kType s2) {
    while (*s && *s==*s2) {s++; s2++;};
    return (*s > *s2) ? 1 : ((*s == *s2) ? 0 : -1);
  }

  bool replaceQ(eType s, eType s2) {return 0;}
};

static _seq<char*> removeDuplicates(_seq<char*> S) {
  return removeDuplicates(S,hashStr());}

template <class intT>
static Table<hashStr,intT> makeStrTable(intT m) {
  return Table<hashStr,intT>(m,hashStr());}

template <class KEYHASH, class DTYPE>
struct hashPair {
  KEYHASH keyHash;
  typedef typename KEYHASH::kType kType;
  typedef pair<kType,DTYPE>* eType;
  eType empty() {return NULL;}

  hashPair(KEYHASH _k) : keyHash(_k) {}

  kType getKey(eType v) { return v->first; }

  uintT hash(kType s) { return keyHash.hash(s);}
  int cmp(kType s, kType s2) { return keyHash.cmp(s, s2);}

  bool replaceQ(eType s, eType s2) {
    return s->second > s2->second;}
};

static _seq<pair<char*,intT>*> removeDuplicates(_seq<pair<char*,intT>*> S) {
  return removeDuplicates(S,hashPair<hashStr,intT>(hashStr()));}

#endif // _A_HASH_INCLUDED
