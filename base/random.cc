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
 */

#include <cstdlib>
#include <cmath>

#include "sim/param.hh"
#include "base/random.hh"
#include "base/trace.hh"

using namespace std;

class RandomContext : public ParamContext
{
  public:
    RandomContext(const string &_iniSection)
        : ::ParamContext(_iniSection) {}
    ~RandomContext() {}

    void checkParams();
};

RandomContext paramContext("random");

Param<unsigned>
seed(&paramContext, "seed", "seed to random number generator", 1);

void
RandomContext::checkParams()
{
    ::srand48(seed);
}

long
getLong()
{
    return mrand48();
}

int64_t
getUniform(int64_t maxmin)
{
    double r;
    r = (drand48() - 0.500) * 2 * maxmin;
    DPRINTFN("getUniform %f\n", r);
    return (int64_t)round(r);
}

uint64_t
getUniformPos(uint64_t max)
{
    double r;
    r = drand48() * 2 * max;
    return (uint64_t)round(r);
}


// idea for generating a double from erand48
double
getDouble()
{
    union {
        uint32_t _long[2];
        uint16_t _short[4];
    };

    _long[0] = mrand48();
    _long[1] = mrand48();

    return ldexp((double) _short[0], -48) +
        ldexp((double) _short[1], -32) +
        ldexp((double) _short[2], -16);
}
