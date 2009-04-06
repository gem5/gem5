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
#include <vector>

#include "base/str.hh"

using namespace std;

int
main(int argc, char *argv[])
{
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " <number>\n";
        exit(1);
    }

    string s = argv[1];

#define OUTVAL(valtype, type) do { \
        valtype value;             \
        cout << "TYPE = " #valtype "\n";        \
        if (to_number(s, value)) {              \
            cout << "Number(" << s << ") = " << dec       \
                 << (unsigned long long)(unsigned type)value << "\n"    \
                 << "Number(" << s << ") = " << dec                     \
                 << (signed long long)(signed type)value << "\n"        \
                 << "Number(" << s << ") = 0x" << hex                   \
                 << (unsigned long long)(unsigned type)value << "\n"    \
                 << "Number(" << s << ") = 0" << oct                    \
                 << (unsigned long long)(unsigned type)value << "\n\n"; \
        } else                                                          \
            cout << "Number(" << s << ") is invalid\n\n";               \
    } while (0)

    OUTVAL(signed long long, long long);
    OUTVAL(unsigned long long, long long);
    OUTVAL(signed long, long);
    OUTVAL(unsigned long, long);
    OUTVAL(signed int, int);
    OUTVAL(unsigned int, int);
    OUTVAL(signed short, short);
    OUTVAL(unsigned short, short);
    OUTVAL(signed char, char);
    OUTVAL(unsigned char, char);

    return 0;
}
