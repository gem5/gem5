/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/**
 * Simple physical register file class.
 * Right now this is specific to Alpha until we decide if/how to make things
 * generic enough to support other ISAs.
 */
template <class Impl>
class PhysRegFile
{
  protected:
    typedef TheISA::IntReg IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::FloatRegBits FloatRegBits;

    typedef union {
        FloatReg d;
        FloatRegBits q;
    } PhysFloatReg;

    // Note that most of the definitions of the IntReg, FloatReg, etc. exist
    // within the Impl/ISA class and not within this PhysRegFile class.

    // Will make these registers public for now, but they probably should
    // be private eventually with some accessor functions.
  public:
    typedef typename Impl::O3CPU O3CPU;

    /**
     * Constructs a physical register file with the specified amount of
     * integer and floating point registers.
     */
    PhysRegFile(O3CPU *_cpu, unsigned _numPhysicalIntRegs,
                unsigned _numPhysicalFloatRegs);

    //Everything below should be pretty well identical to the normal
    //register file that exists within AlphaISA class.
    //The duplication is unfortunate but it's better than having
    //different ways to access certain registers.

    /** Reads an integer register. */
    uint64_t readIntReg(PhysRegIndex reg_idx)
    {
        assert(reg_idx < numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to int register %i, has data "
                "%#x\n", int(reg_idx), intRegFile[reg_idx]);
        return intRegFile[reg_idx];
    }

    /** Reads a floating point register (double precision). */
    FloatReg readFloatReg(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        FloatReg floatReg = floatRegFile[reg_idx].d;

        DPRINTF(IEW, "RegFile: Access to float register %i, has "
                "data %#x\n", int(reg_idx), floatRegFile[reg_idx].q);

        return floatReg;
    }

    FloatRegBits readFloatRegBits(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        FloatRegBits floatRegBits = floatRegFile[reg_idx].q;

        DPRINTF(IEW, "RegFile: Access to float register %i as int, "
                "has data %#x\n", int(reg_idx), (uint64_t)floatRegBits);

        return floatRegBits;
    }

    /** Sets an integer register to the given value. */
    void setIntReg(PhysRegIndex reg_idx, uint64_t val)
    {
        assert(reg_idx < numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting int register %i to %#x\n",
                int(reg_idx), val);

        if (reg_idx != TheISA::ZeroReg)
            intRegFile[reg_idx] = val;
    }

    /** Sets a double precision floating point register to the given value. */
    void setFloatReg(PhysRegIndex reg_idx, FloatReg val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %#x\n",
                int(reg_idx), (uint64_t)val);

#if THE_ISA == ALPHA_ISA
        if (reg_idx != TheISA::ZeroReg)
#endif
            floatRegFile[reg_idx].d = val;
    }

    void setFloatRegBits(PhysRegIndex reg_idx, FloatRegBits val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %#x\n",
                int(reg_idx), (uint64_t)val);

        floatRegFile[reg_idx].q = val;
    }

  public:
    /** (signed) integer register file. */
    IntReg *intRegFile;

    /** Floating point register file. */
    PhysFloatReg *floatRegFile;

  private:
    int intrflag;                       // interrupt flag

  private:
    /** CPU pointer. */
    O3CPU *cpu;

  public:
    /** Number of physical integer registers. */
    unsigned numPhysicalIntRegs;
    /** Number of physical floating point registers. */
    unsigned numPhysicalFloatRegs;
};

template <class Impl>
PhysRegFile<Impl>::PhysRegFile(O3CPU *_cpu, unsigned _numPhysicalIntRegs,
                               unsigned _numPhysicalFloatRegs)
    : cpu(_cpu), numPhysicalIntRegs(_numPhysicalIntRegs),
      numPhysicalFloatRegs(_numPhysicalFloatRegs)
{
    intRegFile = new IntReg[numPhysicalIntRegs];
    floatRegFile = new PhysFloatReg[numPhysicalFloatRegs];

    memset(intRegFile, 0, sizeof(IntReg) * numPhysicalIntRegs);
    memset(floatRegFile, 0, sizeof(PhysFloatReg) * numPhysicalFloatRegs);
}

#endif
