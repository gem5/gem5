/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Ali Saidi
 */

#include <limits>
#include "base/fenv.hh"
#include "base/intmath.hh"
#include "base/misc.hh"
#include "base/random.hh"
#include "sim/serialize.hh"

using namespace std;

Random::Random()
{
    // default random seed taken from original source
    init(5489);
}

Random::Random(uint32_t s)
{
    init(s);
}

Random::Random(uint32_t init_key[], int key_length)
{
    init(init_key, key_length);
}

Random::~Random()
{
}

// To preserve the uniform random distribution between min and max,
// and allow all numbers to be represented, we generate a uniform
// random number to the nearest power of two greater than max.  If
// this number doesn't fall between 0 and max, we try again.  Anything
// else would skew the distribution.
uint32_t
Random::genrand(uint32_t max)
{
    if (max == 0)
        return 0;
    if (max == std::numeric_limits<uint32_t>::max())
        return genrand();

    int log = ceilLog2(max + 1);
    int shift = (sizeof(uint32_t) * 8 - log);
    uint32_t random;

    do {
        random = genrand() >> shift;
    } while (random > max);

    return random;
}

uint64_t
Random::genrand(uint64_t max)
{
    if (max == 0)
        return 0;
    if (max == std::numeric_limits<uint64_t>::max())
        return genrand();

    int log = ceilLog2(max + 1);
    int shift = (sizeof(uint64_t) * 8 - log);
    uint64_t random;

    do {
        random = (uint64_t)genrand() << 32 | (uint64_t)genrand();
        random = random >> shift;
    } while (random > max);

    return random;
}

void
Random::serialize(const string &base, ostream &os)
{
    int length = N;
    paramOut(os, base + ".mti", mti);
    paramOut(os, base + ".length", length);
    arrayParamOut(os, base + ".data", mt, length);
}

void
Random::unserialize(const string &base, Checkpoint *cp, const string &section)
{
    int length;

    paramIn(cp, section, base + ".mti", mti);
    paramIn(cp, section, base + ".length", length);
    if (length != N)
        panic("cant unserialize random number data. length != %d\n", length);

    arrayParamIn(cp, section, base + ".data", mt, length);
}

Random random_mt;
