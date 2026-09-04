/*
 * The Clear BSD License
 *
 * Copyright (c) 2015-2021 Arm Limited.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARM_TLM_HELPERS_H
#define ARM_TLM_HELPERS_H

#include <stdint.h>

namespace ARM
{
namespace TLM
{

/**
 * Wrapper for enumeration values to allow their size (as object data members)
 * to be controlled and to provide a base for attaching functions to enumeration
 * values. Enum-typed values can be extracted from an EnumWrapper with an
 * implicit cast.
 */
template <typename Enum, typename Int = uint32_t>
class EnumWrapper
{
protected:
    /**
     * The wrapped enumeration value held as a raw integer to limit its
     * allocated size.
     */
    Int value;

public:
    /**
     * Default construction with initialized value of 0. It is assumed that 0
     * will be a sensible default value in type Enum
     */
    EnumWrapper() : value(0) {}

    /** Construction from an Enum value. */
    EnumWrapper(Enum value_) : value(static_cast<Int>(value_)) {}

    /** Construction from a raw integer. */
    EnumWrapper(Int value_) : value(value_) {}

    /** Implicit cast to the wrapped enumeration type. */
    operator Enum () const { return static_cast<Enum>(value); }

    /** Assignment from an Enum value. */
    EnumWrapper& operator= (Enum value_) { value = value_; return *this; }
};

/**
 * BitEnumWrapper wraps a value of an enumeration type which can be
 * bitwise-composed from elements of another enumeration type. Elements of the
 * BitEnum type can be bitwise or-ed (with operator|) into a BitEnumWrapper and
 * a composed Enum value can be extracted from the wrapper using an implicit
 * cast.
 *
 * For example:
 *
 * enum Flags { NA_NB = 0, NA_B = 1, A_NB = 2, A_B = 3 };
 *
 * Flags is effectively a 2 bit record of elements A and B.
 * BitEnumWrapper can express this with:
 *
 * enum FlagsBits { A = 2, B = 1 };
 * typedef BitEnumWrapper<Flags, FlagsBits> WrappedFlags.
 *
 * The value A_B of Flags could then be constructed using:
 *
 * WrappedFlags() | A | B
 */
template <typename Enum, typename BitEnum, typename Int = uint32_t>
class BitEnumWrapper
{
protected:
    /**
     * The wrapped enumeration value held as a raw integer to limit its
     * allocated size.
     */
    Int value;

public:
    /**
     * Default construction with initialized value of 0. It is assumed that 0
     * will be a sensible default value in type Enum
     */
    BitEnumWrapper() : value(0) {}

    /** Construction from an Enum value. */
    BitEnumWrapper(Enum value_) : value(static_cast<Int>(value_)) {}

    /** Construction from a raw integer value. */
    BitEnumWrapper(Int value_) : value(value_) {}

    /** Implicit cast to the wrapped enumeration type. */
    operator Enum () const { return static_cast<Enum>(value); }

    /** Assignment from an Enum value. */
    BitEnumWrapper& operator= (Enum value_)
    { value = value_; return *this; }

    /** Bitor in a bitfield from the BitOr type. */
    BitEnumWrapper& operator|= (BitEnum rhs)
    { value = static_cast<Int>(value | rhs); return *this; }

    /**
     * Make a new wrapper from the bitwise or-ed combination of this wrapper and
     * a BitEnum.
     */
    BitEnumWrapper operator| (BitEnum rhs) const
    {
        return BitEnumWrapper<Enum, BitEnum, Int>(
            static_cast<Enum>(value | rhs));
    }

    /**
     * Does this wrapper's value have all the bits set that are set in the
     * given BitEnum. Useful for testing BitEnum flag bit values.
     */
    bool has(BitEnum rhs) { return value & rhs; }

    /** Make a new wrapper with only the bits covered by the given BitEnum. */
    BitEnumWrapper mask(BitEnum rhs) { return value & rhs; }
};

}
}

#endif /* ARM_TLM_HELPERS_H */
