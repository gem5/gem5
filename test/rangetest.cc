/*
 * Copyright (c) 2003 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <string>

#include "base/range.hh"


int
main()
{
  Range<int> r1(9, 28);
  Range<unsigned> r2("0x1000:+0x100");

  cout << r1 << "\n"
       << r2 << "\n";

#define RANGETEST(X, C, Y) \
  cout << X << " "#C" " << Y << " => " << ((X C Y) ? "true" : "false") << "\n"

  int i1 = 10;
  int i2 = 0x1001;
  RANGETEST(i1, < , r1);
  RANGETEST(i1, <=, r1);
  RANGETEST(i1, > , r1);
  RANGETEST(i1, >=, r1);
  RANGETEST(i1, ==, r1);
  RANGETEST(i1, !=, r1);
  RANGETEST(r1, < , i1);
  RANGETEST(r1, <=, i1);
  RANGETEST(r1, > , i1);
  RANGETEST(r1, >=, i1);
  RANGETEST(r1, ==, i1);
  RANGETEST(r1, !=, i1);

  RANGETEST(i2, < , r1);
  RANGETEST(i2, <=, r1);
  RANGETEST(i2, > , r1);
  RANGETEST(i2, >=, r1);
  RANGETEST(i2, ==, r1);
  RANGETEST(i2, !=, r1);
  RANGETEST(r1, < , i2);
  RANGETEST(r1, <=, i2);
  RANGETEST(r1, > , i2);
  RANGETEST(r1, >=, i2);
  RANGETEST(r1, ==, i2);
  RANGETEST(r1, !=, i2);

  unsigned u1 = 10;
  unsigned u2 = 0x1001;
  RANGETEST(u1, < , r2);
  RANGETEST(u1, <=, r2);
  RANGETEST(u1, > , r2);
  RANGETEST(u1, >=, r2);
  RANGETEST(u1, ==, r2);
  RANGETEST(u1, !=, r2);
  RANGETEST(r2, < , u1);
  RANGETEST(r2, <=, u1);
  RANGETEST(r2, > , u1);
  RANGETEST(r2, >=, u1);
  RANGETEST(r2, ==, u1);
  RANGETEST(r2, !=, u1);

  RANGETEST(u2, < , r2);
  RANGETEST(u2, <=, r2);
  RANGETEST(u2, > , r2);
  RANGETEST(u2, >=, r2);
  RANGETEST(u2, ==, r2);
  RANGETEST(u2, !=, r2);
  RANGETEST(r2, < , u2);
  RANGETEST(r2, <=, u2);
  RANGETEST(r2, > , u2);
  RANGETEST(r2, >=, u2);
  RANGETEST(r2, ==, u2);
  RANGETEST(r2, !=, u2);

  return 0;
}
