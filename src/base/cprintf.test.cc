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

#include <gtest/gtest.h>

#include <cstdio>
#include <sstream>
#include <string>

#include "base/cprintf.hh"

#define CPRINTF_TEST(...)                                \
    do {                                                 \
        std::stringstream ss;                            \
        ccprintf(ss, __VA_ARGS__);                       \
        int maxlen = ss.str().length() + 3;              \
        char *buf = new char[maxlen];                    \
        buf[maxlen - 1] = '\0';                          \
        snprintf(buf, maxlen - 2, __VA_ARGS__);          \
        EXPECT_EQ(ss.str(), std::string(buf));           \
        delete [] buf;                                   \
    } while (0)

TEST(CPrintf, Misc)
{
    char foo[] = "foo";
    CPRINTF_TEST("%s\n", foo);

    CPRINTF_TEST("%d\n", 'A');
    CPRINTF_TEST("%shits%%s + %smisses%%s\n", "test", "test");
    CPRINTF_TEST("%%s%-10s %c he went home \'\"%d %#o %#llx %1.5f %1.2E\n",
                 "hello", 'A', 1, 0xff, 0xfffffffffffffULL, 3.141592653589,
                 1.1e10);

    CPRINTF_TEST("another test\n");

    CPRINTF_TEST("%-10s %c he home \'\"%d %#o %#llx %1.5f %1.2E\n",
                 "hello", 'A', 1, 0xff, 0xfffffffffffffULL,
                 3.14159265, 1.1e10);
}

TEST(CPrintf, FloatingPoint)
{
    double f = 314159.26535897932384;

    CPRINTF_TEST("%1.8f\n", f);
    CPRINTF_TEST("%2.8f\n", f);
    CPRINTF_TEST("%3.8f\n", f);
    CPRINTF_TEST("%4.8f\n", f);
    CPRINTF_TEST("%5.8f\n", f);
    CPRINTF_TEST("%6.8f\n", f);
    CPRINTF_TEST("%12.8f\n", f);
    CPRINTF_TEST("%1000.8f\n", f);
    CPRINTF_TEST("%1.0f\n", f);
    CPRINTF_TEST("%1.1f\n", f);
    CPRINTF_TEST("%1.2f\n", f);
    CPRINTF_TEST("%1.3f\n", f);
    CPRINTF_TEST("%1.4f\n", f);
    CPRINTF_TEST("%1.5f\n", f);
    CPRINTF_TEST("%1.6f\n", f);
    CPRINTF_TEST("%1.7f\n", f);
    CPRINTF_TEST("%1.8f\n", f);
    CPRINTF_TEST("%1.9f\n", f);
    CPRINTF_TEST("%1.10f\n", f);
    CPRINTF_TEST("%1.11f\n", f);
    CPRINTF_TEST("%1.12f\n", f);
    CPRINTF_TEST("%1.13f\n", f);
    CPRINTF_TEST("%1.14f\n", f);
    CPRINTF_TEST("%1.15f\n", f);
    CPRINTF_TEST("%1.16f\n", f);
    CPRINTF_TEST("%1.17f\n", f);
    CPRINTF_TEST("%1.18f\n", f);

    f = 0.00000026535897932384;
    CPRINTF_TEST("%1.8f\n", f);
    CPRINTF_TEST("%2.8f\n", f);
    CPRINTF_TEST("%3.8f\n", f);
    CPRINTF_TEST("%4.8f\n", f);
    CPRINTF_TEST("%5.8f\n", f);
    CPRINTF_TEST("%6.8f\n", f);
    CPRINTF_TEST("%12.8f\n", f);
    CPRINTF_TEST("%1.0f\n", f);
    CPRINTF_TEST("%1.1f\n", f);
    CPRINTF_TEST("%1.2f\n", f);
    CPRINTF_TEST("%1.3f\n", f);
    CPRINTF_TEST("%1.4f\n", f);
    CPRINTF_TEST("%1.5f\n", f);
    CPRINTF_TEST("%1.6f\n", f);
    CPRINTF_TEST("%1.7f\n", f);
    CPRINTF_TEST("%1.8f\n", f);
    CPRINTF_TEST("%1.9f\n", f);
    CPRINTF_TEST("%1.10f\n", f);
    CPRINTF_TEST("%1.11f\n", f);
    CPRINTF_TEST("%1.12f\n", f);
    CPRINTF_TEST("%1.13f\n", f);
    CPRINTF_TEST("%1.14f\n", f);
    CPRINTF_TEST("%1.15f\n", f);
    CPRINTF_TEST("%1.16f\n", f);
    CPRINTF_TEST("%1.17f\n", f);
    CPRINTF_TEST("%1.18f\n", f);

    f = 0.00000026535897932384;
    CPRINTF_TEST("%1.8e\n", f);
    CPRINTF_TEST("%2.8e\n", f);
    CPRINTF_TEST("%3.8e\n", f);
    CPRINTF_TEST("%4.8e\n", f);
    CPRINTF_TEST("%5.8e\n", f);
    CPRINTF_TEST("%6.8e\n", f);
    CPRINTF_TEST("%12.8e\n", f);
    CPRINTF_TEST("%1.0e\n", f);
    CPRINTF_TEST("%1.1e\n", f);
    CPRINTF_TEST("%1.2e\n", f);
    CPRINTF_TEST("%1.3e\n", f);
    CPRINTF_TEST("%1.4e\n", f);
    CPRINTF_TEST("%1.5e\n", f);
    CPRINTF_TEST("%1.6e\n", f);
    CPRINTF_TEST("%1.7e\n", f);
    CPRINTF_TEST("%1.8e\n", f);
    CPRINTF_TEST("%1.9e\n", f);
    CPRINTF_TEST("%1.10e\n", f);
    CPRINTF_TEST("%1.11e\n", f);
    CPRINTF_TEST("%1.12e\n", f);
    CPRINTF_TEST("%1.13e\n", f);
    CPRINTF_TEST("%1.14e\n", f);
    CPRINTF_TEST("%1.15e\n", f);
    CPRINTF_TEST("%1.16e\n", f);
    CPRINTF_TEST("%1.17e\n", f);
    CPRINTF_TEST("%1.18e\n", f);
}

TEST(CPrintf, Types)
{
    std::stringstream ss;

    std::string foo1 = "string test";
    ccprintf(ss, "%s\n", foo1);
    EXPECT_EQ(ss.str(), "string test\n");
    ss.str("");

    std::stringstream foo2;
    foo2 << "stringstream test";
    ccprintf(ss, "%s\n", foo2.str());
    EXPECT_EQ(ss.str(), "stringstream test\n");
    ss.str("");

    CPRINTF_TEST("%c  %c\n", 'c', 65);
}

TEST(CPrintf, SpecialFormatting)
{
    CPRINTF_TEST("%08.4f\n", 99.99);
    CPRINTF_TEST("%0*.*f\n", 8, 4, 99.99);
    CPRINTF_TEST("%07.*f\n", 4, 1.234);
    CPRINTF_TEST("%#0*x\n", 9, 123412);
}
