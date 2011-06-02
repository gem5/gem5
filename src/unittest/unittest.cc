/*
 * Copyright (c) 2011 Advanced Micro Devices, Inc.
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
 * Authors: Gabe Black
 */

#include <cstdlib>

#include "base/cprintf.hh"
#include "unittest/unittest.hh"

namespace {

bool _printOnPass = (getenv("PRINT_ON_PASS") != NULL);
unsigned _passes = 0;
unsigned _failures = 0;

bool _casePrinted = false;
const char *_case = NULL;

} // anonymous namespace

namespace UnitTest {

void
checkVal(const char *file, const unsigned line,
         const char *test, const bool result)
{
    if (!result || _printOnPass) {
        if (!_casePrinted && _case) {
            cprintf("CASE %s:\n", _case);
            _casePrinted = true;
        }
        cprintf("   CHECK %s:   %s:%d   %s\n",
                result ? "PASSED" : "FAILED", file, line, test);
    }
    if (result) _passes++;
    else _failures++;
}

bool printOnPass() { return _printOnPass; }
void printOnPass(bool newPrintOnPass) { _printOnPass = newPrintOnPass; }

unsigned passes() { return _passes; }
unsigned failures() { return _failures; }

unsigned
printResults()
{
    cprintf("TEST %s:   %d checks passed, %d checks failed.\n",
            _failures ? "FAILED" : "PASSED", _passes, _failures);
    return _failures;
}

void
reset()
{
    _passes = 0;
    _failures = 0;
}

void
setCase(const char *newCase)
{
    _casePrinted = false;
    _case = newCase;
}

} //namespace UnitTest
