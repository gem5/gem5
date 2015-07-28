/*
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
 * All rights reserved
 *.
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

#ifndef __CPU__REG_CLASS_HH__
#define __CPU__REG_CLASS_HH__

#include <cassert>
#include <cstddef>

#include "arch/registers.hh"
#include "config/the_isa.hh"

/// Enumerate the classes of registers.
enum RegClass {
    IntRegClass,        ///< Integer register
    FloatRegClass,      ///< Floating-point register
    CCRegClass,         ///< Condition-code register
    MiscRegClass        ///< Control (misc) register
};

/// Number of register classes.  This value is not part of the enum,
/// because putting it there makes the compiler complain about
/// unhandled cases in some switch statements.
const int NumRegClasses = MiscRegClass + 1;

/**
 * Map a 'unified' architectural register index to its register class.
 * The unified architectural register index space is used to represent
 * all architectural register identifiers in a single contiguous
 * index space.  See http://gem5.org/Register_Indexing.
 *
 * @param reg_idx Unified-space register index
 * @param rel_reg_idx Optional output param pointer; if non-NULL, location
 *        will be written with the relative register index for reg_idx
 *
 * @return Register class of reg_idx
 */
inline
RegClass regIdxToClass(TheISA::RegIndex reg_idx,
                       TheISA::RegIndex *rel_reg_idx = NULL)
{
    assert(reg_idx < TheISA::Max_Reg_Index);
    RegClass cl;
    int offset;

    if (reg_idx < TheISA::FP_Reg_Base) {
        cl = IntRegClass;
        offset = 0;
    } else if (reg_idx < TheISA::CC_Reg_Base) {
        cl = FloatRegClass;
        offset = TheISA::FP_Reg_Base;
    } else if (reg_idx < TheISA::Misc_Reg_Base) {
        // if there are no CC regs, the ISA should set
        // CC_Reg_Base == Misc_Reg_Base so the if above
        // never succeeds
        cl = CCRegClass;
        offset = TheISA::CC_Reg_Base;
    } else {
        cl = MiscRegClass;
        offset = TheISA::Misc_Reg_Base;
    }

    if (rel_reg_idx)
        *rel_reg_idx = reg_idx - offset;
    return cl;
}

/// Map enum values to strings for debugging
extern const char *RegClassStrings[];


#endif // __CPU__REG_CLASS_HH__
