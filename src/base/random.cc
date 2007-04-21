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

#ifdef __SUNPRO_CC
#include <stdlib.h>
#include <math.h>
#endif

#include <cstdlib>
#include <cmath>

#include "base/fenv.hh"
#include "base/random.hh"

using namespace std;

uint32_t
getInt32()
{
    return mrand48() & 0xffffffff;
}

double
getDouble()
{
    return drand48();
}

double
m5round(double r)
{
#if defined(__sun)
    double val;
    int oldrnd = m5_fegetround();
    m5_fesetround(M5_FE_TONEAREST);
    val = rint(r);
    m5_fesetround(oldrnd);
    return val;
#else
    return round(r);
#endif
}

int64_t
getUniform(int64_t min, int64_t max)
{
    double r;
    r = drand48() * (max-min) + min;

    return (int64_t)m5round(r);
}

uint64_t
getUniformPos(uint64_t min, uint64_t max)
{
    double r;
    r = drand48() * (max-min) + min;

    return (uint64_t)m5round(r);
}
