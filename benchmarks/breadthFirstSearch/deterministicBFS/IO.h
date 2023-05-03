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

#ifndef _BENCH_IO
#define _BENCH_IO

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include "sequence.h"
#include "parallel.h"

namespace benchIO {
  using namespace std;

  // A structure that keeps a sequence of strings all allocated from
  // the same block of memory
  struct words {
    long n; // total number of characters
    char* Chars;  // array storing all strings
    long m; // number of substrings
    char** Strings; // pointers to strings (all should be null terminated)
    words() {}
    words(char* C, long nn, char** S, long mm)
      : Chars(C), n(nn), Strings(S), m(mm) {}
    void del() {free(Chars); free(Strings);}
  };

  inline bool isSpace(char c) {
    switch (c)  {
    case '\r':
    case '\t':
    case '\n':
    case 0:
    case ' ' : return true;
    default : return false;
    }
  }

  struct toLong { long operator() (bool v) {return (long) v;} };

  // parallel code for converting a string to words
  words stringToWords(char *Str, long n) {
    parallel_for (long i=0; i < n; i++)
      if (isSpace(Str[i])) Str[i] = 0;

    // mark start of words
    bool *FL = newA(bool,n);
    FL[0] = Str[0];
    parallel_for (long i=1; i < n; i++) FL[i] = Str[i] && !Str[i-1];

    // offset for each start of word
    _seq<long> Off = sequence::packIndex(FL, n);
    long m = Off.n;
    long *offsets = Off.A;

    // pointer to each start of word
    char **SA = newA(char*, m);
    parallel_for (long j=0; j < m; j++) SA[j] = Str+offsets[j];

    free(offsets); free(FL);
    return words(Str,n,SA,m);
  }

  int writeStringToFile(char* S, long n, char* fileName) {
    ofstream file (fileName, ios::out | ios::binary);
    if (!file.is_open()) {
      std::cout << "Unable to open file: " << fileName << std::endl;
      return 1;
    }
    file.write(S, n);
    file.close();
    return 0;
  }

  inline int xToStringLen(long a) { return 21;}
  inline void xToString(char* s, long a) { sprintf(s,"%ld",a);}

  inline int xToStringLen(int a) { return 12;}
  inline void xToString(char* s, int a) { sprintf(s,"%d",a);}

  inline int xToStringLen(double a) { return 18;}
  inline void xToString(char* s, double a) { sprintf(s,"%.11le", a);}

  inline int xToStringLen(char* a) { return strlen(a)+1;}
  inline void xToString(char* s, char* a) { sprintf(s,"%s",a);}

  template <class A, class B>
  inline int xToStringLen(pair<A,B> a) {
    return xToStringLen(a.first) + xToStringLen(a.second) + 1;
  }
  template <class A, class B>
  inline void xToString(char* s, pair<A,B> a) {
    int l = xToStringLen(a.first);
    xToString(s,a.first);
    s[l] = ' ';
    xToString(s+l+1,a.second);
  }

  struct notZero { bool operator() (char A) {return A > 0;}};

  template <class T>
  _seq<char> arrayToString(T* A, long n) {
    long* L = newA(long,n);
    {parallel_for(long i=0; i < n; i++) L[i] = xToStringLen(A[i])+1;}
    long m = sequence::scan(L,L,n,utils::addF<long>(),(long) 0);
    char* B = newA(char,m);
    parallel_for(long j=0; j < m; j++)
      B[j] = 0;
    parallel_for(long i=0; i < n-1; i++) {
      xToString(B + L[i],A[i]);
      B[L[i+1] - 1] = '\n';
    }
    xToString(B + L[n-1],A[n-1]);
    B[m-1] = '\n';
    free(L);
    char* C = newA(char,m+1);
    long mm = sequence::filter(B,C,m,notZero());
    C[mm] = 0;
    free(B);
    return _seq<char>(C,mm);
  }

  template <class T>
  void writeArrayToStream(ofstream& os, T* A, long n) {
    long BSIZE = 1000000;
    long offset = 0;
    while (offset < n) {
      // Generates a string for a sequence of size at most BSIZE
      // and then wrties it to the output stream
      _seq<char> S = arrayToString(A+offset,min(BSIZE,n-offset));
      os.write(S.A, S.n);
      S.del();
      offset += BSIZE;
    }
  }

  template <class T>
    int writeArrayToFile(string header, T* A, long n, char* fileName) {
    ofstream file (fileName, ios::out | ios::binary);
    if (!file.is_open()) {
      std::cout << "Unable to open file: " << fileName << std::endl;
      return 1;
    }
    file << header << endl;
    writeArrayToStream(file, A, n);
    file.close();
    return 0;
  }

  _seq<char> readStringFromFile(char *fileName) {
    ifstream file (fileName, ios::in | ios::binary | ios::ate);
    if (!file.is_open()) {
      std::cout << "Unable to open file: " << fileName << std::endl;
      abort();
    }
    long end = file.tellg();
    file.seekg (0, ios::beg);
    long n = end - file.tellg();
    char* bytes = newA(char,n+1);
    file.read (bytes,n);
    file.close();
    return _seq<char>(bytes,n);
  }

  string intHeaderIO = "sequenceInt";

  template <class intT>
  intT writeIntArrayToFile(intT* A, long n, char* fileName) {
    return writeArrayToFile(intHeaderIO, A, n, fileName);
  }

  template <class intT>
  _seq<intT> readIntArrayFromFile(char *fileName) {
    _seq<char> S = readStringFromFile(fileName);
    words W = stringToWords(S.A, S.n);
    string header = (string) W.Strings[0];
    if (header != intHeaderIO) {
      cout << "readIntArrayFromFile: bad input" << endl;
      abort();
    }
    long n = W.m-1;
    intT* A = new intT[n];
    parallel_for(long i=0; i < n; i++)
      A[i] = atol(W.Strings[i+1]);
    return _seq<intT>(A,n);
  }
};

#endif // _BENCH_IO
