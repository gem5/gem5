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

#ifndef __CPU_O3_CPU_REGFILE_HH__
#define __CPU_O3_CPU_REGFILE_HH__

// @todo: Destructor

#include "arch/isa_traits.hh"
#include "arch/faults.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/o3/comm.hh"

#if FULL_SYSTEM
#include "kern/kernel_stats.hh"

#endif

// This really only depends on the ISA, and not the Impl.  It might be nicer
// to see if I can make it depend on nothing...
// Things that are in the ifdef FULL_SYSTEM are pretty dependent on the ISA,
// and should go in the AlphaFullCPU.

template <class Impl>
class PhysRegFile
{
  protected:
    typedef TheISA::IntReg IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::MiscRegFile MiscRegFile;
    typedef TheISA::MiscReg MiscReg;

    //Note that most of the definitions of the IntReg, FloatReg, etc. exist
    //within the Impl/ISA class and not within this PhysRegFile class.

    //Will need some way to allow stuff like swap_palshadow to access the
    //correct registers.  Might require code changes to swap_palshadow and
    //other execution contexts.

    //Will make these registers public for now, but they probably should
    //be private eventually with some accessor functions.
  public:
    typedef typename Impl::FullCPU FullCPU;

    PhysRegFile(unsigned _numPhysicalIntRegs,
                unsigned _numPhysicalFloatRegs);

    //Everything below should be pretty well identical to the normal
    //register file that exists within AlphaISA class.
    //The duplication is unfortunate but it's better than having
    //different ways to access certain registers.

    //Add these in later when everything else is in place
//    void serialize(std::ostream &os);
//    void unserialize(Checkpoint *cp, const std::string &section);

    uint64_t readIntReg(PhysRegIndex reg_idx)
    {
        assert(reg_idx < numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to int register %i, has data "
                "%i\n", int(reg_idx), intRegFile[reg_idx]);
        return intRegFile[reg_idx];
    }

    FloatReg readFloatReg(PhysRegIndex reg_idx, int width)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        FloatReg floatReg = floatRegFile.readReg(reg_idx, width);

        DPRINTF(IEW, "RegFile: Access to %d byte float register %i, has "
                "data %8.8d\n", int(reg_idx), (double)floatReg);

        return floatReg;
    }

    FloatReg readFloatReg(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        FloatReg floatReg = floatRegFile.readReg(reg_idx);

        DPRINTF(IEW, "RegFile: Access to float register %i, has "
                "data %8.8d\n", int(reg_idx), (double)floatReg);

        return floatReg;
    }

    FloatRegBits readFloatRegBits(PhysRegIndex reg_idx, int width)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        FloatRegBits floatRegBits = floatRegFile.readRegBits(reg_idx, width);

        DPRINTF(IEW, "RegFile: Access to %d byte float register %i as int, "
                "has data %lli\n", int(reg_idx), (uint64_t)floatRegBits);

        return floatRegBits;
    }

    FloatRegBits readFloatRegBits(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        FloatRegBits floatRegBits = floatRegFile.readRegBits(reg_idx);

        DPRINTF(IEW, "RegFile: Access to float register %i as int, "
                "has data %lli\n", int(reg_idx), (uint64_t)floatRegBits);

        return floatRegBits;
    }

    void setIntReg(PhysRegIndex reg_idx, uint64_t val)
    {
        assert(reg_idx < numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting int register %i to %lli\n",
                int(reg_idx), val);

        intRegFile[reg_idx] = val;
    }

    void setFloatReg(PhysRegIndex reg_idx, FloatReg val, int width)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %8.8d\n",
                int(reg_idx), (double)val);

        floatRegFile.setReg(reg_idx, val, width);
    }

    void setFloatReg(PhysRegIndex reg_idx, FloatReg val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %8.8d\n",
                int(reg_idx), (double)val);

        floatRegFile.setReg(reg_idx, val);
    }

    void setFloatRegBits(PhysRegIndex reg_idx, FloatRegBits val, int width)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %lli\n",
                int(reg_idx), (uint64_t)val);

        floatRegFile.setRegBits(reg_idx, val, width);
    }

    void setFloatRegBits(PhysRegIndex reg_idx, FloatRegBits val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %lli\n",
                int(reg_idx), (uint64_t)val);

        floatRegFile.setRegBits(reg_idx, val);
    }

    uint64_t readPC()
    {
        return pc;
    }

    void setPC(uint64_t val)
    {
        pc = val;
    }

    void setNextPC(uint64_t val)
    {
        npc = val;
    }

    //Consider leaving this stuff and below in some implementation specific
    //file as opposed to the general register file.  Or have a derived class.
    MiscReg readMiscReg(int misc_reg)
    {
        // Dummy function for now.
        // @todo: Fix this once proxy XC is used.
        return 0;
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val)
    {
        // Dummy function for now.
        // @todo: Fix this once proxy XC is used.
        return NoFault;
    }

#if FULL_SYSTEM
    int readIntrFlag() { return intrflag; }
    void setIntrFlag(int val) { intrflag = val; }
#endif

    // These should be private eventually, but will be public for now
    // so that I can hack around the initregs issue.
  public:
    /** (signed) integer register file. */
    IntReg *intRegFile;

    /** Floating point register file. */
    FloatReg *floatRegFile;

    /** Miscellaneous register file. */
    MiscRegFile miscRegs;

    /** Program counter. */
    Addr pc;

    /** Next-cycle program counter. */
    Addr npc;

#if FULL_SYSTEM
  private:
    // This is ISA specifc stuff; remove it eventually once ISAImpl is used
//    IntReg palregs[NumIntRegs];	// PAL shadow registers
    int intrflag;			// interrupt flag
    bool pal_shadow;		// using pal_shadow registers
#endif

  private:
    FullCPU *cpu;

  public:
    void setCPU(FullCPU *cpu_ptr) { cpu = cpu_ptr; }

    unsigned numPhysicalIntRegs;
    unsigned numPhysicalFloatRegs;
};

template <class Impl>
PhysRegFile<Impl>::PhysRegFile(unsigned _numPhysicalIntRegs,
                               unsigned _numPhysicalFloatRegs)
    : numPhysicalIntRegs(_numPhysicalIntRegs),
      numPhysicalFloatRegs(_numPhysicalFloatRegs)
{
    intRegFile = new IntReg[numPhysicalIntRegs];
    floatRegFile = new FloatReg[numPhysicalFloatRegs];

    memset(intRegFile, 0, sizeof(*intRegFile));
    memset(floatRegFile, 0, sizeof(*floatRegFile));
}

#endif // __CPU_O3_CPU_REGFILE_HH__
