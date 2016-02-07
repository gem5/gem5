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
#include <list>
#include <sstream>
#include <string>

#include "base/cprintf.hh"
#include "base/misc.hh"

using namespace std;

int
main()
{
    char foo[] = "foo";
    cprintf("%s\n", foo);

    string _bar = "asdfkhasdlkfjhasdlkfhjalksdjfhalksdjhfalksdjfhalksdjhf";
    int length = 11;
    char bar[length + 1];
    bar[length] = 0;

    memcpy(bar, _bar.c_str(), length);
    warn("%s\n", bar);

    cprintf("%d\n", 'A');
    cprintf("%shits%%s + %smisses%%s\n", "test", "test");
    cprintf("%%s%-10s %c he went home \'\"%d %#o %#x %1.5f %1.2E\n",
            "hello", 'A', 1, 0xff, 0xfffffffffffffULL, 3.141592653589, 1.1e10);

    cprintf("another test\n");

    stringstream buffer;
    ccprintf(buffer, "%-10s %c he home \'\"%d %#o %#x %1.5f %1.2E\n",
             "hello", 'A', 1, 0xff, 0xfffffffffffffULL, 3.14159265, 1.1e10);

    double f = 314159.26535897932384;

    #define ctest(x, y) printf(x, y); cprintf(x, y); cprintf("\n");
    ctest("%1.8f\n", f);
    ctest("%2.8f\n", f);
    ctest("%3.8f\n", f);
    ctest("%4.8f\n", f);
    ctest("%5.8f\n", f);
    ctest("%6.8f\n", f);
    ctest("%12.8f\n", f);
    ctest("%1000.8f\n", f);
    ctest("%1.0f\n", f);
    ctest("%1.1f\n", f);
    ctest("%1.2f\n", f);
    ctest("%1.3f\n", f);
    ctest("%1.4f\n", f);
    ctest("%1.5f\n", f);
    ctest("%1.6f\n", f);
    ctest("%1.7f\n", f);
    ctest("%1.8f\n", f);
    ctest("%1.9f\n", f);
    ctest("%1.10f\n", f);
    ctest("%1.11f\n", f);
    ctest("%1.12f\n", f);
    ctest("%1.13f\n", f);
    ctest("%1.14f\n", f);
    ctest("%1.15f\n", f);
    ctest("%1.16f\n", f);
    ctest("%1.17f\n", f);
    ctest("%1.18f\n", f);

    cout << "foo\n";

    f = 0.00000026535897932384;
    ctest("%1.8f\n", f);
    ctest("%2.8f\n", f);
    ctest("%3.8f\n", f);
    ctest("%4.8f\n", f);
    ctest("%5.8f\n", f);
    ctest("%6.8f\n", f);
    ctest("%12.8f\n", f);
    ctest("%1.0f\n", f);
    ctest("%1.1f\n", f);
    ctest("%1.2f\n", f);
    ctest("%1.3f\n", f);
    ctest("%1.4f\n", f);
    ctest("%1.5f\n", f);
    ctest("%1.6f\n", f);
    ctest("%1.7f\n", f);
    ctest("%1.8f\n", f);
    ctest("%1.9f\n", f);
    ctest("%1.10f\n", f);
    ctest("%1.11f\n", f);
    ctest("%1.12f\n", f);
    ctest("%1.13f\n", f);
    ctest("%1.14f\n", f);
    ctest("%1.15f\n", f);
    ctest("%1.16f\n", f);
    ctest("%1.17f\n", f);
    ctest("%1.18f\n", f);

    f = 0.00000026535897932384;
    ctest("%1.8e\n", f);
    ctest("%2.8e\n", f);
    ctest("%3.8e\n", f);
    ctest("%4.8e\n", f);
    ctest("%5.8e\n", f);
    ctest("%6.8e\n", f);
    ctest("%12.8e\n", f);
    ctest("%1.0e\n", f);
    ctest("%1.1e\n", f);
    ctest("%1.2e\n", f);
    ctest("%1.3e\n", f);
    ctest("%1.4e\n", f);
    ctest("%1.5e\n", f);
    ctest("%1.6e\n", f);
    ctest("%1.7e\n", f);
    ctest("%1.8e\n", f);
    ctest("%1.9e\n", f);
    ctest("%1.10e\n", f);
    ctest("%1.11e\n", f);
    ctest("%1.12e\n", f);
    ctest("%1.13e\n", f);
    ctest("%1.14e\n", f);
    ctest("%1.15e\n", f);
    ctest("%1.16e\n", f);
    ctest("%1.17e\n", f);
    ctest("%1.18e\n", f);

    cout << buffer.str();

    cout.width(0);
    cout.precision(1);
    cout << f << "\n";

    string foo1 = "string test";
    cprintf("%s\n", foo1);

    stringstream foo2;
    foo2 << "stringstream test";
    cprintf("%s\n", foo2);

    cprintf("%c  %c\n", 'c', 65);

    cout << '9' << endl;

    cout << endl;

    cprintf("%08.4f\n", 99.99);
    cprintf("%0*.*f\n", 8, 4, 99.99);
    cprintf("%07.*f\n", 4, 1.234);
    cprintf("%#0*x\n", 9, 123412);
    return 0;
}
