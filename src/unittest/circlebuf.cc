/*
 * Copyright (c) 2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 * Authors: Andreas Sandberg
 */

#include "base/circlebuf.hh"
#include "unittest/unittest.hh"

const char data[] = {
    0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
    0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf,
};

int
main(int argc, char *argv[])
{
    UnitTest::setCase("Basic non-overflow functionality");
    {
        CircleBuf<char> buf(8);
        char foo[16];

        // Write empty buffer, no overflow
        buf.write(data, 8);
        EXPECT_EQ(buf.size(), 8);
        buf.peek(foo, 8);
        EXPECT_EQ(memcmp(foo, data, 8), 0);

        // Read 2
        buf.read(foo, 2);
        EXPECT_EQ(memcmp(foo, data, 2), 0);
        EXPECT_EQ(buf.size(), 6);
        buf.read(foo, 6);
        EXPECT_EQ(memcmp(foo, data + 2, 6), 0);
        EXPECT_EQ(buf.size(), 0);
    }

    UnitTest::setCase("Basic single write overflow functionality");
    {
        CircleBuf<char> buf(8);
        char foo[16];

        buf.write(data, 16);
        EXPECT_EQ(buf.size(), 8);
        buf.peek(foo, 8);
        EXPECT_EQ(memcmp(data + 8, foo, 8), 0);
    }


    UnitTest::setCase("Multi-write overflow functionality");
    {
        CircleBuf<char> buf(8);
        char foo[16];

        // Write, no overflow, write overflow
        buf.write(data, 6);
        buf.write(data + 8, 6);
        EXPECT_EQ(buf.size(), 8);
        buf.peek(foo, 8);
        EXPECT_EQ(memcmp(data + 4, foo, 2), 0);
        EXPECT_EQ(memcmp(data + 8, foo + 2, 6), 0);
    }

    UnitTest::setCase("Pointer wrap around");
    {
        CircleBuf<char> buf(8);
        char foo[16];

        // _start == 0, _stop = 8
        buf.write(data, 8);
        // _start == 4, _stop = 8
        buf.read(foo, 4);
        // _start == 4, _stop = 12
        buf.write(data + 8, 4);
        EXPECT_EQ(buf.size(), 8);
        // _start == 10, _stop = 12
        // Normalized: _start == 2, _stop = 4
        buf.read(foo + 4, 6);
        EXPECT_EQ(buf.size(), 2);
        EXPECT_EQ(memcmp(data, foo, 10), 0);
        // Normalized: _start == 4, _stop = 4
        buf.read(foo + 10, 2);
        EXPECT_EQ(buf.size(), 0);
        EXPECT_EQ(memcmp(data, foo, 12), 0);
    }

    return UnitTest::printResults();
}
