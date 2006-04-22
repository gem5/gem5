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
 */

#ifndef __CPU_O3_REGFILE_HH__
#define __CPU_O3_REGFILE_HH__

#include "arch/isa_traits.hh"
#include "arch/faults.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/o3/comm.hh"

#if FULL_SYSTEM
#include "kern/kernel_stats.hh"

#endif

#include <vector>

/**
 * Simple physical register file class.
 * This really only depends on the ISA, and not the Impl. Things that are
 * in the ifdef FULL_SYSTEM are pretty dependent on the ISA, and probably
 * should go in the AlphaFullCPU.
 */
template <class Impl>
class PhysRegFile
{
  protected:
    typedef TheISA::IntReg IntReg;
    typedef TheISA::FloatReg FloatReg;
    typedef TheISA::MiscRegFile MiscRegFile;
    typedef TheISA::MiscReg MiscReg;
    // Note that most of the definitions of the IntReg, FloatReg, etc. exist
    // within the Impl/ISA class and not within this PhysRegFile class.

    // Will make these registers public for now, but they probably should
    // be private eventually with some accessor functions.
  public:
    typedef typename Impl::FullCPU FullCPU;

    /**
     * Constructs a physical register file with the specified amount of
     * integer and floating point registers.
     */
    PhysRegFile(unsigned _numPhysicalIntRegs,
                unsigned _numPhysicalFloatRegs);

    //Everything below should be pretty well identical to the normal
    //register file that exists within AlphaISA class.
    //The duplication is unfortunate but it's better than having
    //different ways to access certain registers.

    //Add these in later when everything else is in place
//    void serialize(std::ostream &os);
//    void unserialize(Checkpoint *cp, const std::string &section);

    /** Reads an integer register. */
    uint64_t readIntReg(PhysRegIndex reg_idx)
    {
        assert(reg_idx < numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to int register %i, has data "
                "%i\n", int(reg_idx), intRegFile[reg_idx]);
        return intRegFile[reg_idx];
    }

    /** Reads a floating point register (single precision). */
    float readFloatRegSingle(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to float register %i as single, has "
                "data %8.8f\n", int(reg_idx), (float)floatRegFile[reg_idx].d);

        return (float)floatRegFile[reg_idx].d;
    }

    /** Reads a floating point register (double precision). */
    double readFloatRegDouble(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to float register %i as double, has "
                " data %8.8f\n", int(reg_idx), floatRegFile[reg_idx].d);

        return floatRegFile[reg_idx].d;
    }

    /** Reads a floating point register as an integer. */
    uint64_t readFloatRegInt(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to float register %i as int, has data "
                "%lli\n", int(reg_idx), floatRegFile[reg_idx].q);

        return floatRegFile[reg_idx].q;
    }

    /** Sets an integer register to the given value. */
    void setIntReg(PhysRegIndex reg_idx, uint64_t val)
    {
        assert(reg_idx < numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting int register %i to %lli\n",
                int(reg_idx), val);

        if (reg_idx != TheISA::ZeroReg)
            intRegFile[reg_idx] = val;
    }

    /** Sets a single precision floating point register to the given value. */
    void setFloatRegSingle(PhysRegIndex reg_idx, float val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %8.8f\n",
                int(reg_idx), val);

        if (reg_idx != TheISA::ZeroReg)
            floatRegFile[reg_idx].d = (double)val;
    }

    /** Sets a double precision floating point register to the given value. */
    void setFloatRegDouble(PhysRegIndex reg_idx, double val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %8.8f\n",
                int(reg_idx), val);

        if (reg_idx != TheISA::ZeroReg)
            floatRegFile[reg_idx].d = val;
    }

    /** Sets a floating point register to the given integer value. */
    void setFloatRegInt(PhysRegIndex reg_idx, uint64_t val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %lli\n",
                int(reg_idx), val);

        if (reg_idx != TheISA::ZeroReg)
            floatRegFile[reg_idx].q = val;
    }

    //Consider leaving this stuff and below in some implementation specific
    //file as opposed to the general register file.  Or have a derived class.
    MiscReg readMiscReg(int misc_reg, unsigned thread_id)
    {
        return miscRegs[thread_id].readReg(misc_reg);
    }

    MiscReg readMiscRegWithEffect(int misc_reg, Fault &fault,
                                  unsigned thread_id)
    {
        return miscRegs[thread_id].readRegWithEffect(misc_reg, fault,
                                                     cpu->xcProxies[thread_id]);
    }

    Fault setMiscReg(int misc_reg, const MiscReg &val, unsigned thread_id)
    {
        return miscRegs[thread_id].setReg(misc_reg, val);
    }

    Fault setMiscRegWithEffect(int misc_reg, const MiscReg &val,
                               unsigned thread_id)
    {
        return miscRegs[thread_id].setRegWithEffect(misc_reg, val,
                                                    cpu->xcProxies[thread_id]);
    }

#if FULL_SYSTEM
    int readIntrFlag() { return intrflag; }
    /** Sets an interrupt flag. */
    void setIntrFlag(int val) { intrflag = val; }
#endif

  public:
    /** (signed) integer register file. */
    std::vector<IntReg> intRegFile;

    /** Floating point register file. */
    std::vector<FloatReg> floatRegFile;

    /** Miscellaneous register file. */
    MiscRegFile miscRegs[Impl::MaxThreads];

#if FULL_SYSTEM
  private:
    int intrflag;			// interrupt flag
#endif

  private:
    /** CPU pointer. */
    FullCPU *cpu;

  public:
    /** Sets the CPU pointer. */
    void setCPU(FullCPU *cpu_ptr) { cpu = cpu_ptr; }

    /** Number of physical integer registers. */
    unsigned numPhysicalIntRegs;
    /** Number of physical floating point registers. */
    unsigned numPhysicalFloatRegs;
};

template <class Impl>
PhysRegFile<Impl>::PhysRegFile(unsigned _numPhysicalIntRegs,
                               unsigned _numPhysicalFloatRegs)
    : numPhysicalIntRegs(_numPhysicalIntRegs),
      numPhysicalFloatRegs(_numPhysicalFloatRegs)
{
    intRegFile.resize(numPhysicalIntRegs);
    floatRegFile.resize(numPhysicalFloatRegs);

    //memset(intRegFile, 0, sizeof(*intRegFile));
    //memset(floatRegFile, 0, sizeof(*floatRegFile));
}

#endif
