/*
 * Copyright (c) 2000-2005 The Regents of The University of Michigan
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
 * Authors: Steve Reinhardt
 */

/*
 * This code was originally written by Steve Reinhardt as part of
 * the Wisconsin Wind Tunnel simulator.  Relicensed as part of M5
 * by permission.
 */

#include <cassert>

#include "base/fast_alloc.hh"

#if USE_FAST_ALLOC

void *FastAlloc::freeLists[Num_Buckets];

#if FAST_ALLOC_STATS
unsigned FastAlloc::newCount[Num_Buckets];
unsigned FastAlloc::deleteCount[Num_Buckets];
unsigned FastAlloc::allocCount[Num_Buckets];
#endif

void *
FastAlloc::moreStructs(int bucket)
{
    assert(bucket > 0 && bucket < Num_Buckets);

    int sz = bucket * Alloc_Quantum;
    const int nstructs = Num_Structs_Per_New;   // how many to allocate?
    char *p = ::new char[nstructs * sz];

#if FAST_ALLOC_STATS
    ++allocCount[bucket];
#endif

    freeLists[bucket] = p;
    for (int i = 0; i < (nstructs-2); ++i, p += sz)
        *(void **)p = p + sz;
    *(void **)p = 0;

    return (p + sz);
}

#endif // USE_FAST_ALLOC
