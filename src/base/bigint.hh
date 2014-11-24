/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#include <iostream>

#include "base/misc.hh"
#include "base/types.hh"

#ifndef __BASE_BIGINT_HH__
#define __BASE_BIGINT_HH__
// Create a couple of large int types for atomic reads
struct m5_twin64_t {
    uint64_t a;
    uint64_t b;
    m5_twin64_t() : a(0), b(0)
    {}
    m5_twin64_t(const uint64_t x) : a(x), b(x)
    {}
    inline m5_twin64_t& operator=(const uint64_t x)
    {
        a = x;
        b = x;
        return *this;
    }

    operator uint64_t()
    {
        panic("Tried to cram a twin64_t into an integer!\n");
        return a;
    }
};

struct m5_twin32_t {
    uint32_t a;
    uint32_t b;
    m5_twin32_t()
    {}
    m5_twin32_t(const uint32_t x)
    {
        a = x;
        b = x;
    }
    inline m5_twin32_t& operator=(const uint32_t x)
    {
        a = x;
        b = x;
        return *this;
    }

    operator uint32_t()
    {
        panic("Tried to cram a twin32_t into an integer!\n");
        return a;
    }
};


// This is for twin loads (two 64 bit values), not 1 128 bit value (as far as
// endian conversion is concerned!
typedef m5_twin64_t Twin64_t;
typedef m5_twin32_t Twin32_t;

// Output operator overloads
std::ostream & operator << (std::ostream & os, const Twin64_t & t);
std::ostream & operator << (std::ostream & os, const Twin32_t & t);

#endif // __BASE_BIGINT_HH__

