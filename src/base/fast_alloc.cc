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

#if !NO_FAST_ALLOC

#ifdef __GNUC__
#pragma implementation
#endif

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

#if FAST_ALLOC_DEBUG

#include <map>
#include <string>
#include <typeinfo>

#include "base/cprintf.hh"
#include "sim/core.hh"   // for curTick()

using namespace std;

// count of in-use FastAlloc objects
int FastAlloc::numInUse;

// dummy head & tail object for doubly linked list of in-use FastAlloc
// objects
FastAlloc FastAlloc::inUseHead(&FastAlloc::inUseHead, &FastAlloc::inUseHead);

// special constructor for dummy head: make inUsePrev & inUseNext
// point to self
FastAlloc::FastAlloc(FastAlloc *prev, FastAlloc *next)
{
    inUsePrev = prev;
    inUseNext = next;
}

// constructor: marks as in use, add to in-use list
FastAlloc::FastAlloc()
{
    // mark this object in use
    inUse = true;
    whenAllocated = curTick();

    // update count
    ++numInUse;

    // add to tail of list of in-use objects ("before" dummy head)
    FastAlloc *myNext = &inUseHead;
    FastAlloc *myPrev = inUseHead.inUsePrev;

    inUsePrev = myPrev;
    inUseNext = myNext;
    myPrev->inUseNext = this;
    myNext->inUsePrev = this;
}

// destructor: mark not in use, remove from in-use list
FastAlloc::~FastAlloc()
{
    assert(inUse);
    inUse = false;

    --numInUse;
    assert(numInUse >= 0);

    // remove me from in-use list
    inUsePrev->inUseNext = inUseNext;
    inUseNext->inUsePrev = inUsePrev;
}


// Note that in all the display functions below we suppress anything
// with a zero allocation timestamp... there are a bunch of static or
// quasi-static structures that get allocated during initialization
// and we generally don't care about them so this gets them out of the
// way.

// summarize in-use list
void
FastAlloc::dump_summary()
{
    map<string, int> typemap;

    for (FastAlloc *p = inUseHead.inUseNext; p != &inUseHead; p = p->inUseNext)
    {
        if (p->whenAllocated != 0)
            ++typemap[typeid(*p).name()];
    }

    map<string, int>::const_iterator mapiter;

    cprintf(" count  type\n"
            " -----  ----\n");
    for (mapiter = typemap.begin(); mapiter != typemap.end(); ++mapiter)
        cprintf("%6d  %s\n",mapiter->second, mapiter->first);
}


// show oldest n items on in-use list
void
FastAlloc::dump_oldest(int n)
{
    // sanity check: don't want to crash the debugger if you forget to
    // pass in a parameter
    if (n < 0 || n > numInUse) {
        cprintf("FastAlloc::dump_oldest: bad arg %d (%d objects in use)\n",
                n, numInUse);
        return;
    }

    for (FastAlloc *p = inUseHead.inUseNext;
         p != &inUseHead && n > 0;
         p = p->inUseNext, --n) {
        if (p->whenAllocated != 0)
            cprintf("%x %15d %s\n", p, p->whenAllocated, typeid(*p).name());
    }
}


// show oldest n items on in-use list for specified type
void
FastAlloc::dump_oldest_of_type(int n, const char *type_name)
{
    // sanity check: don't want to crash the debugger if you forget to
    // pass in a parameter
    if (n < 0 || n > numInUse) {
        cprintf("FastAlloc::dump_oldest_of_type: bad arg %d "
                "(%d objects in use)\n",
                n, numInUse);
        return;
    }

    for (FastAlloc *p = inUseHead.inUseNext;
         p != &inUseHead && n > 0;
         p = p->inUseNext) {
        if (p->whenAllocated != 0 &&
            strcmp(typeid(*p).name(), type_name) == 0) {
            cprintf("%x %15d\n", p, p->whenAllocated);
            --n;
        }
    }
}


//
// C interfaces to FastAlloc::dump_summary() and FastAlloc::dump_oldest().
// gdb seems to have trouble with calling C++ functions directly.
//
void
fast_alloc_summary()
{
    FastAlloc::dump_summary();
}

void
fast_alloc_oldest(int n)
{
    FastAlloc::dump_oldest(n);
}

#endif // FAST_ALLOC_DEBUG

#endif // NO_FAST_ALLOC
