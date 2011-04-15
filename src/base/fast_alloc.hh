/*
 * Copyright (c) 2000-2001, 2003-2005 The Regents of The University of Michigan
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

#ifndef __BASE_FAST_ALLOC_HH__
#define __BASE_FAST_ALLOC_HH__

#include <cstddef>

// Fast structure allocator.  Designed for small objects that are
// frequently allocated and deallocated.  This code is derived from the
// 'alloc_struct' package used in WWT and Blizzard.  C++ provides a
// much nicer framework for the same optimization.  The package is
// implemented as a class, FastAlloc.  Allocation and deletion are
// performed using FastAlloc's new and delete operators.  Any object
// that derives from the FastAlloc class will transparently use this
// allocation package.

// The static allocate() and deallocate() methods can also be called
// directly if desired.

// In order for derived classes to call delete with the correct
// structure size even when they are deallocated via a base-type
// pointer, they must have a virtual destructor.  It is sufficient for
// FastAlloc to declare a virtual destructor (as it does); it is not
// required for derived classes to declare their own destructor.  The
// compiler will automatically generate a virtual destructor for each
// derived class.  However, it is more efficient if each derived class
// defines an inline destructor, so that the compiler can statically
// collapse the destructor call chain back up the inheritance
// hierarchy.

#include "config/fast_alloc_stats.hh"
#include "config/force_fast_alloc.hh"
#include "config/no_fast_alloc.hh"

// By default, we want to enable FastAlloc in any build other than
// m5.debug.  (FastAlloc's reuse policies can mask allocation bugs, so
// we typically want it disabled when debugging.)  Set
// FORCE_FAST_ALLOC to enable even when debugging, and set
// NO_FAST_ALLOC to disable even in non-debug builds.
#define USE_FAST_ALLOC \
    (FORCE_FAST_ALLOC || (!defined(DEBUG) && !NO_FAST_ALLOC))

#if !USE_FAST_ALLOC

class FastAlloc
{
};

#else

class FastAlloc
{
  public:
    static void *allocate(size_t);
    static void deallocate(void *, size_t);

    void *operator new(size_t);
    void operator delete(void *, size_t);

    virtual ~FastAlloc() {}

  private:

    // Max_Alloc_Size is the largest object that can be allocated with
    // this class.  There's no fundamental limit, but this limits the
    // size of the freeLists array.  Let's not make this really huge
    // like in Blizzard.
    static const size_t Max_Alloc_Size = 512;

    // Alloc_Quantum is the difference in size between adjacent
    // buckets in the free list array.
    static const int Log2_Alloc_Quantum = 3;
    static const int Alloc_Quantum = (1 << Log2_Alloc_Quantum);

    // Num_Buckets = bucketFor(Max_Alloc_Size) + 1
    static const int Num_Buckets =
        ((Max_Alloc_Size + Alloc_Quantum - 1) >> Log2_Alloc_Quantum) + 1;

    // when we call new() for more structures, how many should we get?
    static const int Num_Structs_Per_New = 20;

    static int bucketFor(size_t);
    static void *moreStructs(int bucket);

    static void *freeLists[Num_Buckets];

#if FAST_ALLOC_STATS
    static unsigned newCount[Num_Buckets];
    static unsigned deleteCount[Num_Buckets];
    static unsigned allocCount[Num_Buckets];
#endif
};

inline int
FastAlloc::bucketFor(size_t sz)
{
    return (sz + Alloc_Quantum - 1) >> Log2_Alloc_Quantum;
}

inline void *
FastAlloc::allocate(size_t sz)
{
    int b;
    void *p;

    if (sz > Max_Alloc_Size)
        return (void *)::new char[sz];

    b = bucketFor(sz);
    p = freeLists[b];

    if (p)
        freeLists[b] = *(void **)p;
    else
        p = moreStructs(b);

#if FAST_ALLOC_STATS
    ++newCount[b];
#endif

    return p;
}

inline void
FastAlloc::deallocate(void *p, size_t sz)
{
    int b;

    if (sz > Max_Alloc_Size) {
        ::delete [] (char *)p;
        return;
    }

    b = bucketFor(sz);
    *(void **)p = freeLists[b];
    freeLists[b] = p;
#if FAST_ALLOC_STATS
    ++deleteCount[b];
#endif
}

inline void *
FastAlloc::operator new(size_t sz)
{
    return allocate(sz);
}

inline void
FastAlloc::operator delete(void *p, size_t sz)
{
    deallocate(p, sz);
}

#endif // USE_FAST_ALLOC

#endif // __BASE_FAST_ALLOC_HH__
