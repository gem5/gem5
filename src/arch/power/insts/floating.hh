/*
 * Copyright (c) 2009 The University of Edinburgh
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

#ifndef __ARCH_POWER_INSTS_FLOATING_HH__
#define __ARCH_POWER_INSTS_FLOATING_HH__

#include "arch/power/insts/static_inst.hh"
#include "base/bitfield.hh"
#include "base/cprintf.hh"

namespace gem5
{

namespace PowerISA
{

/**
 * Base class for floating point operations.
 */
class FloatOp : public PowerStaticInst
{
  protected:

    bool rc;

    /// Constructor
    FloatOp(const char *mnem, MachInst _machInst, OpClass __opClass)
      : PowerStaticInst(mnem, _machInst, __opClass),
        rc(machInst.rc)
    {
    }

    // Test for NaN (maximum biased exponent & non-zero fraction)
    inline bool
    isNan(uint32_t val_bits) const
    {
        return ((bits(val_bits, 30, 23) == 0xFF) && bits(val_bits, 22, 0));
    }

    inline bool
    isNan(uint64_t val_bits) const
    {
        return ((bits(val_bits, 62, 52) == 0x7FF) && bits(val_bits, 51, 0));
    }

    inline bool
    isNan(float val) const
    {
        void *val_ptr = &val;
        uint32_t val_bits = *(uint32_t *) val_ptr;
        return isNan(val_bits);
    }

    inline bool
    isNan(double val) const
    {
        void *val_ptr = &val;
        uint64_t val_bits = *(uint64_t *) val_ptr;
        return isNan(val_bits);
    }

    // Test for SNaN (NaN with high order bit of fraction set to 0)
    inline bool
    isSnan(uint32_t val_bits) const
    {
        return ((bits(val_bits, 30, 22) == 0x1FE) && bits(val_bits, 22, 0));
    }

    // Test for QNaN (NaN with high order bit of fraction set to 1)
    inline bool
    isQnan(uint32_t val_bits) const
    {
        return (bits(val_bits, 30, 22) == 0x1FF);
    }

    // Test for infinity (maximum biased exponent and zero fraction)
    inline bool
    isInfinity(uint32_t val_bits) const
    {
        return ((bits(val_bits, 30, 23) == 0xFF) && !bits(val_bits, 22, 0));
    }

    // Test for normalized numbers (biased exponent in the range 1 to 254)
    inline bool
    isNormalized(uint32_t val_bits) const
    {
        return ((bits(val_bits, 30, 23) != 0xFF) && bits(val_bits, 22, 0));
    }

    // Test for denormalized numbers (biased exponent of zero and
    // non-zero fraction)
    inline bool
    isDenormalized(uint32_t val_bits) const
    {
        return (!bits(val_bits, 30, 23) && bits(val_bits, 22, 0));
    }

    // Test for zero (biased exponent of zero and fraction of zero)
    inline bool
    isZero(uint32_t val_bits) const
    {
        return (!bits(val_bits, 30, 23) && !bits(val_bits, 22, 0));
    }

    // Test for negative
    inline bool
    isNegative(uint32_t val_bits) const
    {
        return (bits(val_bits, 31));
    }

    // Compute the CR field
    inline uint32_t
    makeCRField(double a, double b) const
    {
        uint32_t c = 0;
        if (isNan(a) || isNan(b)) { c = 0x1; }
        else if (a < b)           { c = 0x8; }
        else if (a > b)           { c = 0x4; }
        else                      { c = 0x2; }
        return c;
    }

    std::string generateDisassembly(
            Addr pc, const loader::SymbolTable *symtab) const override;
};

} // namespace PowerISA
} // namespace gem5

#endif //__ARCH_POWER_INSTS_FLOATING_HH__
