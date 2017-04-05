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
    std::vector<PhysRegId> intRegIds;

    /** Floating point register file. */
    std::vector<PhysFloatReg> floatRegFile;
    std::vector<PhysRegId> floatRegIds;

    /** Condition-code register file. */
    std::vector<CCReg> ccRegFile;
    std::vector<PhysRegId> ccRegIds;

    /** Misc Reg Ids */
    std::vector<PhysRegId> miscRegIds;

    /**
     * Number of physical general purpose registers
     */
    unsigned numPhysicalIntRegs;

    /**
     * Number of physical general purpose registers
     */
    unsigned numPhysicalFloatRegs;

    /**
     * Number of physical general purpose registers
     */
    unsigned numPhysicalCCRegs;

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
    unsigned numIntPhysRegs() const { return numPhysicalIntRegs; }

    /** @return the number of floating-point physical registers. */
    unsigned numFloatPhysRegs() const { return numPhysicalFloatRegs; }

    /** @return the number of condition-code physical registers. */
    unsigned numCCPhysRegs() const { return numPhysicalCCRegs; }

    /** @return the total number of physical registers. */
    unsigned totalNumPhysRegs() const { return totalNumRegs; }

    /** Gets a misc register PhysRegIdPtr. */
    PhysRegIdPtr getMiscRegId(RegIndex reg_idx) {
        return &miscRegIds[reg_idx];
    }

    /** Reads an integer register. */
    uint64_t readIntReg(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isIntPhysReg());

        DPRINTF(IEW, "RegFile: Access to int register %i, has data "
                "%#x\n", phys_reg->index(), intRegFile[phys_reg->index()]);
        return intRegFile[phys_reg->index()];
    }

    /** Reads a floating point register (double precision). */
    FloatReg readFloatReg(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isFloatPhysReg());

        DPRINTF(IEW, "RegFile: Access to float register %i, has "
                "data %#x\n", phys_reg->index(),
                floatRegFile[phys_reg->index()].q);

        return floatRegFile[phys_reg->index()].d;
    }

    FloatRegBits readFloatRegBits(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isFloatPhysReg());

        FloatRegBits floatRegBits = floatRegFile[phys_reg->index()].q;

        DPRINTF(IEW, "RegFile: Access to float register %i as int, "
                "has data %#x\n", phys_reg->index(),
                (uint64_t)floatRegBits);

        return floatRegBits;
    }

    /** Reads a condition-code register. */
    CCReg readCCReg(PhysRegIdPtr phys_reg)
    {
        assert(phys_reg->isCCPhysReg());

        DPRINTF(IEW, "RegFile: Access to cc register %i, has "
                "data %#x\n", phys_reg->index(),
                ccRegFile[phys_reg->index()]);

        return ccRegFile[phys_reg->index()];
    }

    /** Sets an integer register to the given value. */
    void setIntReg(PhysRegIdPtr phys_reg, uint64_t val)
    {
        assert(phys_reg->isIntPhysReg());

        DPRINTF(IEW, "RegFile: Setting int register %i to %#x\n",
                phys_reg->index(), val);

        if (!phys_reg->isZeroReg())
            intRegFile[phys_reg->index()] = val;
    }

    /** Sets a double precision floating point register to the given value. */
    void setFloatReg(PhysRegIdPtr phys_reg, FloatReg val)
    {
        assert(phys_reg->isFloatPhysReg());

        DPRINTF(IEW, "RegFile: Setting float register %i to %#x\n",
                phys_reg->index(), (uint64_t)val);

        if (!phys_reg->isZeroReg())
            floatRegFile[phys_reg->index()].d = val;
    }

    void setFloatRegBits(PhysRegIdPtr phys_reg, FloatRegBits val)
    {
        assert(phys_reg->isFloatPhysReg());

        DPRINTF(IEW, "RegFile: Setting float register %i to %#x\n",
                phys_reg->index(), (uint64_t)val);

        floatRegFile[phys_reg->index()].q = val;
    }

    /** Sets a condition-code register to the given value. */
    void setCCReg(PhysRegIdPtr phys_reg, CCReg val)
    {
        assert(phys_reg->isCCPhysReg());

        DPRINTF(IEW, "RegFile: Setting cc register %i to %#x\n",
                phys_reg->index(), (uint64_t)val);

        ccRegFile[phys_reg->index()] = val;
    }
};


#endif //__CPU_O3_REGFILE_HH__
