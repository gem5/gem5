/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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
 * Authors: Kevin Lim
 *          Gabe Black
 */

#ifndef __CPU_O3_REGFILE_HH__
#define __CPU_O3_REGFILE_HH__

#include <vector>

#include "arch/isa_traits.hh"
#include "arch/kernel_stats.hh"
#include "arch/types.hh"
#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/o3/comm.hh"
#include "debug/IEW.hh"

class UnifiedFreeList;

/**
 * Simple physical register file class.
 */
class PhysRegFile
{
  private:

    typedef TheISA::IntReg IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;
    typedef TheISA::CCReg CCReg;

    typedef union {
        FloatReg d;
        FloatRegBits q;
    } PhysFloatReg;

    /** Integer register file. */
    std::vector<IntReg> intRegFile;

    /** Floating point register file. */
    std::vector<PhysFloatReg> floatRegFile;

    /** Condition-code register file. */
    std::vector<CCReg> ccRegFile;

    /**
     * The first floating-point physical register index.  The physical
     * register file has a single continuous index space, with the
     * initial indices mapping to the integer registers, followed
     * immediately by the floating-point registers.  Thus the first
     * floating-point index is equal to the number of integer
     * registers.
     *
     * Note that this internal organizational detail on how physical
     * register file indices are ordered should *NOT* be exposed
     * outside of this class.  Other classes can use the is*PhysReg()
     * methods to map from a physical register index to a class
     * without knowing the internal structure of the index map.
     */
    unsigned baseFloatRegIndex;

    /**
     * The first condition-code physical register index.  The
     * condition-code registers follow the floating-point registers.
     */
    unsigned baseCCRegIndex;

    /** Total number of physical registers. */
    unsigned totalNumRegs;

  public:
    /**
     * Constructs a physical register file with the specified amount of
     * integer and floating point registers.
     */
    PhysRegFile(unsigned _numPhysicalIntRegs,
                unsigned _numPhysicalFloatRegs,
                unsigned _numPhysicalCCRegs);

    /**
     * Destructor to free resources
     */
    ~PhysRegFile() {}

    /** Initialize the free list */
    void initFreeList(UnifiedFreeList *freeList);

    /** @return the number of integer physical registers. */
    unsigned numIntPhysRegs() const { return baseFloatRegIndex; }

    /** @return the number of floating-point physical registers. */
    unsigned numFloatPhysRegs() const
    { return baseCCRegIndex - baseFloatRegIndex; }

    /** @return the number of condition-code physical registers. */
    unsigned numCCPhysRegs() const
    { return totalNumRegs - baseCCRegIndex; }

    /** @return the total number of physical registers. */
    unsigned totalNumPhysRegs() const { return totalNumRegs; }

    /**
     * @return true if the specified physical register index
     * corresponds to an integer physical register.
     */
    bool isIntPhysReg(PhysRegIndex reg_idx) const
    {
        return 0 <= reg_idx && reg_idx < baseFloatRegIndex;
    }

    /**
     * @return true if the specified physical register index
     * corresponds to a floating-point physical register.
     */
    bool isFloatPhysReg(PhysRegIndex reg_idx) const
    {
        return (baseFloatRegIndex <= reg_idx && reg_idx < baseCCRegIndex);
    }

    /**
     * Return true if the specified physical register index
     * corresponds to a condition-code physical register.
     */
    bool isCCPhysReg(PhysRegIndex reg_idx)
    {
        return (baseCCRegIndex <= reg_idx && reg_idx < totalNumRegs);
    }

    /** Reads an integer register. */
    uint64_t readIntReg(PhysRegIndex reg_idx) const
    {
        assert(isIntPhysReg(reg_idx));

        DPRINTF(IEW, "RegFile: Access to int register %i, has data "
                "%#x\n", int(reg_idx), intRegFile[reg_idx]);
        return intRegFile[reg_idx];
    }

    /** Reads a floating point register (double precision). */
    FloatReg readFloatReg(PhysRegIndex reg_idx) const
    {
        assert(isFloatPhysReg(reg_idx));

        // Remove the base Float reg dependency.
        PhysRegIndex reg_offset = reg_idx - baseFloatRegIndex;

        DPRINTF(IEW, "RegFile: Access to float register %i, has "
                "data %#x\n", int(reg_idx), floatRegFile[reg_offset].q);

        return floatRegFile[reg_offset].d;
    }

    FloatRegBits readFloatRegBits(PhysRegIndex reg_idx) const
    {
        assert(isFloatPhysReg(reg_idx));

        // Remove the base Float reg dependency.
        PhysRegIndex reg_offset = reg_idx - baseFloatRegIndex;

        FloatRegBits floatRegBits = floatRegFile[reg_offset].q;

        DPRINTF(IEW, "RegFile: Access to float register %i as int, "
                "has data %#x\n", int(reg_idx), (uint64_t)floatRegBits);

        return floatRegBits;
    }

    /** Reads a condition-code register. */
    CCReg readCCReg(PhysRegIndex reg_idx)
    {
        assert(isCCPhysReg(reg_idx));

        // Remove the base CC reg dependency.
        PhysRegIndex reg_offset = reg_idx - baseCCRegIndex;

        DPRINTF(IEW, "RegFile: Access to cc register %i, has "
                "data %#x\n", int(reg_idx), ccRegFile[reg_offset]);

        return ccRegFile[reg_offset];
    }

    /** Sets an integer register to the given value. */
    void setIntReg(PhysRegIndex reg_idx, uint64_t val)
    {
        assert(isIntPhysReg(reg_idx));

        DPRINTF(IEW, "RegFile: Setting int register %i to %#x\n",
                int(reg_idx), val);

        if (reg_idx != TheISA::ZeroReg)
            intRegFile[reg_idx] = val;
    }

    /** Sets a double precision floating point register to the given value. */
    void setFloatReg(PhysRegIndex reg_idx, FloatReg val)
    {
        assert(isFloatPhysReg(reg_idx));

        // Remove the base Float reg dependency.
        PhysRegIndex reg_offset = reg_idx - baseFloatRegIndex;

        DPRINTF(IEW, "RegFile: Setting float register %i to %#x\n",
                int(reg_idx), (uint64_t)val);

#if THE_ISA == ALPHA_ISA
        if (reg_offset != TheISA::ZeroReg)
#endif
            floatRegFile[reg_offset].d = val;
    }

    void setFloatRegBits(PhysRegIndex reg_idx, FloatRegBits val)
    {
        assert(isFloatPhysReg(reg_idx));

        // Remove the base Float reg dependency.
        PhysRegIndex reg_offset = reg_idx - baseFloatRegIndex;

        DPRINTF(IEW, "RegFile: Setting float register %i to %#x\n",
                int(reg_idx), (uint64_t)val);

        floatRegFile[reg_offset].q = val;
    }

    /** Sets a condition-code register to the given value. */
    void setCCReg(PhysRegIndex reg_idx, CCReg val)
    {
        assert(isCCPhysReg(reg_idx));

        // Remove the base CC reg dependency.
        PhysRegIndex reg_offset = reg_idx - baseCCRegIndex;

        DPRINTF(IEW, "RegFile: Setting cc register %i to %#x\n",
                int(reg_idx), (uint64_t)val);

        ccRegFile[reg_offset] = val;
    }
};


#endif //__CPU_O3_REGFILE_HH__
