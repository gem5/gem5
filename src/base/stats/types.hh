/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

#ifndef __BASE_STATS_TYPES_HH__
#define __BASE_STATS_TYPES_HH__

#include <limits>
#include <map>
#include <vector>

#include "base/compiler.hh"
#include "base/types.hh"

namespace gem5
{

namespace statistics
{

/** All counters are of 64-bit values. */
typedef double Counter;
/** vector of counters. */
typedef std::vector<Counter> VCounter;
/** map of counters */
typedef std::map<Counter, int> MCounter;

typedef std::numeric_limits<Counter> CounterLimits;

/** All results are doubles. */
typedef double Result;
/** vector of results. */
typedef std::vector<Result> VResult;

typedef unsigned int size_type;
typedef unsigned int off_type;

enum DistType { Deviation, Dist, Hist };

/** General container for distribution data. */
struct DistData
{
    DistType type;
    Counter min;
    Counter max;
    Counter bucket_size;

    Counter min_val;
    Counter max_val;
    Counter underflow;
    Counter overflow;
    VCounter cvec;
    Counter sum;
    Counter squares;
    Counter logs;
    Counter samples;
};

/** Data structure of sparse histogram */
struct SparseHistData
{
    MCounter cmap;
    Counter samples;
};

} // namespace statistics
} // namespace gem5

#endif // __BASE_STATS_TYPES_HH__
