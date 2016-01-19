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
 */

/**
 * @file
 * Defines global host-dependent types:
 * Counter, Tick, and (indirectly) {int,uint}{8,16,32,64}_t.
 */

#ifndef __BASE_TYPES_HH__
#define __BASE_TYPES_HH__

#include <inttypes.h>

#include <cassert>
#include <memory>
#include <ostream>
#include <stdexcept>

#include "base/refcnt.hh"

/** uint64_t constant */
#define ULL(N)          ((uint64_t)N##ULL)
/** int64_t constant */
#define LL(N)           ((int64_t)N##LL)

/** Statistics counter type.  Not much excuse for not using a 64-bit
 * integer here, but if you're desperate and only run short
 * simulations you could make this 32 bits.
 */
typedef int64_t Counter;

/**
 * Tick count type.
 */
typedef uint64_t Tick;

const Tick MaxTick = ULL(0xffffffffffffffff);

/**
 * Cycles is a wrapper class for representing cycle counts, i.e. a
 * relative difference between two points in time, expressed in a
 * number of clock cycles.
 *
 * The Cycles wrapper class is a type-safe alternative to a
 * typedef, aiming to avoid unintentional mixing of cycles and ticks
 * in the code base.
 *
 * Operators are defined inside an ifndef block to avoid swig touching
 * them. Note that there is no overloading of the bool operator as the
 * compiler is allowed to turn booleans into integers and this causes
 * a whole range of issues in a handful locations. The solution to
 * this problem would be to use the safe bool idiom, but for now we
 * make do without the test and use the more elaborate comparison >
 * Cycles(0).
 */
class Cycles
{

  private:

    /** Member holding the actual value. */
    uint64_t c;

  public:

#ifndef SWIG // SWIG gets confused by constexpr
    /** Explicit constructor assigning a value. */
    explicit constexpr Cycles(uint64_t _c) : c(_c) { }
#else
    explicit Cycles(uint64_t _c) : c(_c) { }
#endif

    /** Default constructor for parameter classes. */
    Cycles() : c(0) { }

#ifndef SWIG // keep the operators away from SWIG

    /** Converting back to the value type. */
    constexpr operator uint64_t() const { return c; }

    /** Prefix increment operator. */
    Cycles& operator++()
    { ++c; return *this; }

    /** Prefix decrement operator. Is only temporarily used in the O3 CPU. */
    Cycles& operator--()
    { assert(c != 0); --c; return *this; }

    /** In-place addition of cycles. */
    Cycles& operator+=(const Cycles& cc)
    { c += cc.c; return *this; }

    /** Greater than comparison used for > Cycles(0). */
    constexpr bool operator>(const Cycles& cc) const
    { return c > cc.c; }

    constexpr Cycles operator +(const Cycles& b) const
    { return Cycles(c + b.c); }

    constexpr Cycles operator -(const Cycles& b) const
    {
        return c >= b.c ? Cycles(c - b.c) :
            throw std::invalid_argument("RHS cycle value larger than LHS");
    }

    constexpr Cycles operator <<(const int32_t shift) const
    { return Cycles(c << shift); }

    constexpr Cycles operator >>(const int32_t shift) const
    { return Cycles(c >> shift); }

    friend std::ostream& operator<<(std::ostream &out, const Cycles & cycles);

#endif // SWIG not touching operators

};

/**
 * Address type
 * This will probably be moved somewhere else in the near future.
 * This should be at least as big as the biggest address width in use
 * in the system, which will probably be 64 bits.
 */
typedef uint64_t Addr;

typedef uint16_t MicroPC;

static const MicroPC MicroPCRomBit = 1 << (sizeof(MicroPC) * 8 - 1);

static inline MicroPC
romMicroPC(MicroPC upc)
{
    return upc | MicroPCRomBit;
}

static inline MicroPC
normalMicroPC(MicroPC upc)
{
    return upc & ~MicroPCRomBit;
}

static inline bool
isRomMicroPC(MicroPC upc)
{
    return MicroPCRomBit & upc;
}

const Addr MaxAddr = (Addr)-1;

/**
 * Thread index/ID type
 */
typedef int16_t ThreadID;
const ThreadID InvalidThreadID = (ThreadID)-1;

/** Globally unique thread context ID */
typedef int ContextID;
const ContextID InvalidContextID = (ContextID)-1;

/**
 * Port index/ID type, and a symbolic name for an invalid port id.
 */
typedef int16_t PortID;
const PortID InvalidPortID = (PortID)-1;

class FaultBase;
typedef std::shared_ptr<FaultBase> Fault;

#ifndef SWIG // Swig gets really confused by decltype
// Rather than creating a shared_ptr instance and assigning it nullptr,
// we just create an alias.
constexpr decltype(nullptr) NoFault = nullptr;
#endif

struct AtomicOpFunctor
{
    virtual void operator()(uint8_t *p) = 0;
    virtual ~AtomicOpFunctor() {}
};

template <class T>
struct TypedAtomicOpFunctor : public AtomicOpFunctor
{
    void operator()(uint8_t *p) { execute((T *)p); }
    virtual void execute(T * p) = 0;
};

enum ByteOrder {
    BigEndianByteOrder,
    LittleEndianByteOrder
};

#endif // __BASE_TYPES_HH__
