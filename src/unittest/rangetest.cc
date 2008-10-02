/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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
 *
 * Authors: Nathan Binkert
 */

#include <iostream>
#include <string>

#include "base/range.hh"

using namespace std;

int
main()
{
    Range<int> r1(make_pair(9, 28));
    Range<unsigned> r2("0x1000:+0x100");

    cout << r1 << "\n"
         << r2 << "\n";

#define RANGETEST(X, C, Y)                                              \
    cout << X << " "#C" " << Y << " => " <<                             \
        ((X C Y) ? "true" : "false") << "\n"

#define TESTEM(X, Y) do { \
        RANGETEST(X, < , Y);                    \
        RANGETEST(X, <=, Y);                    \
        RANGETEST(X, > , Y);                    \
        RANGETEST(X, >=, Y);                    \
        RANGETEST(X, ==, Y);                    \
        RANGETEST(X, !=, Y);                    \
        RANGETEST(Y, < , X);                    \
        RANGETEST(Y, <=, X);                    \
        RANGETEST(Y, > , X);                    \
        RANGETEST(Y, >=, X);                    \
        RANGETEST(Y, ==, X);                    \
        RANGETEST(Y, !=, X);                    \
    } while (0)

    TESTEM(8, r1);
    TESTEM(9, r1);
    TESTEM(27, r1);
    TESTEM(28, r1);

    TESTEM(0x0fff, r2);
    TESTEM(0x1000, r2);
    TESTEM(0x10ff, r2);
    TESTEM(0x1100, r2);

    return 0;
}
