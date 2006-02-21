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

#ifndef __CPU_O3_CPU_REGFILE_HH__
#define __CPU_O3_CPU_REGFILE_HH__

// @todo: Destructor

#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/faults.hh"
#include "base/trace.hh"
#include "config/full_system.hh"
#include "cpu/o3/comm.hh"

#if FULL_SYSTEM
#include "arch/alpha/ev5.hh"
#include "kern/kernel_stats.hh"

using namespace EV5;
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

    float readFloatRegSingle(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to float register %i as single, has "
                "data %8.8f\n", int(reg_idx), (float)floatRegFile[reg_idx].d);

        return (float)floatRegFile[reg_idx].d;
    }

    double readFloatRegDouble(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to float register %i as double, has "
                " data %8.8f\n", int(reg_idx), floatRegFile[reg_idx].d);

        return floatRegFile[reg_idx].d;
    }

    uint64_t readFloatRegInt(PhysRegIndex reg_idx)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Access to float register %i as int, has data "
                "%lli\n", int(reg_idx), floatRegFile[reg_idx].q);

        return floatRegFile[reg_idx].q;
    }

    void setIntReg(PhysRegIndex reg_idx, uint64_t val)
    {
        assert(reg_idx < numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting int register %i to %lli\n",
                int(reg_idx), val);

        intRegFile[reg_idx] = val;
    }

    void setFloatRegSingle(PhysRegIndex reg_idx, float val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %8.8f\n",
                int(reg_idx), val);

        floatRegFile[reg_idx].d = (double)val;
    }

    void setFloatRegDouble(PhysRegIndex reg_idx, double val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %8.8f\n",
                int(reg_idx), val);

        floatRegFile[reg_idx].d = val;
    }

    void setFloatRegInt(PhysRegIndex reg_idx, uint64_t val)
    {
        // Remove the base Float reg dependency.
        reg_idx = reg_idx - numPhysicalIntRegs;

        assert(reg_idx < numPhysicalFloatRegs + numPhysicalIntRegs);

        DPRINTF(IEW, "RegFile: Setting float register %i to %lli\n",
                int(reg_idx), val);

        floatRegFile[reg_idx].q = val;
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
    uint64_t readUniq()
    {
        return miscRegs.uniq;
    }

    void setUniq(uint64_t val)
    {
        miscRegs.uniq = val;
    }

    uint64_t readFpcr()
    {
        return miscRegs.fpcr;
    }

    void setFpcr(uint64_t val)
    {
        miscRegs.fpcr = val;
    }

#if FULL_SYSTEM
    uint64_t readIpr(int idx, Fault * &fault);
    Fault * setIpr(int idx, uint64_t val);
    InternalProcReg *getIpr() { return ipr; }
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
    IntReg palregs[NumIntRegs];	// PAL shadow registers
    InternalProcReg ipr[NumInternalProcRegs]; // internal processor regs
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

#if FULL_SYSTEM

//Problem:  This code doesn't make sense at the RegFile level because it
//needs things such as the itb and dtb.  Either put it at the CPU level or
//the DynInst level.
template <class Impl>
uint64_t
PhysRegFile<Impl>::readIpr(int idx, Fault * &fault)
{
    uint64_t retval = 0;    // return value, default 0

    switch (idx) {
      case TheISA::IPR_PALtemp0:
      case TheISA::IPR_PALtemp1:
      case TheISA::IPR_PALtemp2:
      case TheISA::IPR_PALtemp3:
      case TheISA::IPR_PALtemp4:
      case TheISA::IPR_PALtemp5:
      case TheISA::IPR_PALtemp6:
      case TheISA::IPR_PALtemp7:
      case TheISA::IPR_PALtemp8:
      case TheISA::IPR_PALtemp9:
      case TheISA::IPR_PALtemp10:
      case TheISA::IPR_PALtemp11:
      case TheISA::IPR_PALtemp12:
      case TheISA::IPR_PALtemp13:
      case TheISA::IPR_PALtemp14:
      case TheISA::IPR_PALtemp15:
      case TheISA::IPR_PALtemp16:
      case TheISA::IPR_PALtemp17:
      case TheISA::IPR_PALtemp18:
      case TheISA::IPR_PALtemp19:
      case TheISA::IPR_PALtemp20:
      case TheISA::IPR_PALtemp21:
      case TheISA::IPR_PALtemp22:
      case TheISA::IPR_PALtemp23:
      case TheISA::IPR_PAL_BASE:

      case TheISA::IPR_IVPTBR:
      case TheISA::IPR_DC_MODE:
      case TheISA::IPR_MAF_MODE:
      case TheISA::IPR_ISR:
      case TheISA::IPR_EXC_ADDR:
      case TheISA::IPR_IC_PERR_STAT:
      case TheISA::IPR_DC_PERR_STAT:
      case TheISA::IPR_MCSR:
      case TheISA::IPR_ASTRR:
      case TheISA::IPR_ASTER:
      case TheISA::IPR_SIRR:
      case TheISA::IPR_ICSR:
      case TheISA::IPR_ICM:
      case TheISA::IPR_DTB_CM:
      case TheISA::IPR_IPLR:
      case TheISA::IPR_INTID:
      case TheISA::IPR_PMCTR:
        // no side-effect
        retval = ipr[idx];
        break;

      case TheISA::IPR_CC:
        retval |= ipr[idx] & ULL(0xffffffff00000000);
        retval |= curTick  & ULL(0x00000000ffffffff);
        break;

      case TheISA::IPR_VA:
        retval = ipr[idx];
        break;

      case TheISA::IPR_VA_FORM:
      case TheISA::IPR_MM_STAT:
      case TheISA::IPR_IFAULT_VA_FORM:
      case TheISA::IPR_EXC_MASK:
      case TheISA::IPR_EXC_SUM:
        retval = ipr[idx];
        break;

      case TheISA::IPR_DTB_PTE:
        {
            TheISA::PTE &pte = cpu->dtb->index(1);

            retval |= ((u_int64_t)pte.ppn & ULL(0x7ffffff)) << 32;
            retval |= ((u_int64_t)pte.xre & ULL(0xf)) << 8;
            retval |= ((u_int64_t)pte.xwe & ULL(0xf)) << 12;
            retval |= ((u_int64_t)pte.fonr & ULL(0x1)) << 1;
            retval |= ((u_int64_t)pte.fonw & ULL(0x1))<< 2;
            retval |= ((u_int64_t)pte.asma & ULL(0x1)) << 4;
            retval |= ((u_int64_t)pte.asn & ULL(0x7f)) << 57;
        }
        break;

        // write only registers
      case TheISA::IPR_HWINT_CLR:
      case TheISA::IPR_SL_XMIT:
      case TheISA::IPR_DC_FLUSH:
      case TheISA::IPR_IC_FLUSH:
      case TheISA::IPR_ALT_MODE:
      case TheISA::IPR_DTB_IA:
      case TheISA::IPR_DTB_IAP:
      case TheISA::IPR_ITB_IA:
      case TheISA::IPR_ITB_IAP:
        fault = UnimplementedOpcodeFault;
        break;

      default:
        // invalid IPR
        fault = UnimplementedOpcodeFault;
        break;
    }

    return retval;
}

extern int break_ipl;

template <class Impl>
Fault *
PhysRegFile<Impl>::setIpr(int idx, uint64_t val)
{
    uint64_t old;

    switch (idx) {
      case TheISA::IPR_PALtemp0:
      case TheISA::IPR_PALtemp1:
      case TheISA::IPR_PALtemp2:
      case TheISA::IPR_PALtemp3:
      case TheISA::IPR_PALtemp4:
      case TheISA::IPR_PALtemp5:
      case TheISA::IPR_PALtemp6:
      case TheISA::IPR_PALtemp7:
      case TheISA::IPR_PALtemp8:
      case TheISA::IPR_PALtemp9:
      case TheISA::IPR_PALtemp10:
      case TheISA::IPR_PALtemp11:
      case TheISA::IPR_PALtemp12:
      case TheISA::IPR_PALtemp13:
      case TheISA::IPR_PALtemp14:
      case TheISA::IPR_PALtemp15:
      case TheISA::IPR_PALtemp16:
      case TheISA::IPR_PALtemp17:
      case TheISA::IPR_PALtemp18:
      case TheISA::IPR_PALtemp19:
      case TheISA::IPR_PALtemp20:
      case TheISA::IPR_PALtemp21:
      case TheISA::IPR_PALtemp22:
      case TheISA::IPR_PAL_BASE:
      case TheISA::IPR_IC_PERR_STAT:
      case TheISA::IPR_DC_PERR_STAT:
      case TheISA::IPR_PMCTR:
        // write entire quad w/ no side-effect
        ipr[idx] = val;
        break;

      case TheISA::IPR_CC_CTL:
        // This IPR resets the cycle counter.  We assume this only
        // happens once... let's verify that.
        assert(ipr[idx] == 0);
        ipr[idx] = 1;
        break;

      case TheISA::IPR_CC:
        // This IPR only writes the upper 64 bits.  It's ok to write
        // all 64 here since we mask out the lower 32 in rpcc (see
        // isa_desc).
        ipr[idx] = val;
        break;

      case TheISA::IPR_PALtemp23:
        // write entire quad w/ no side-effect
        old = ipr[idx];
        ipr[idx] = val;
        break;

      case TheISA::IPR_DTB_PTE:
        // write entire quad w/ no side-effect, tag is forthcoming
        ipr[idx] = val;
        break;

      case TheISA::IPR_EXC_ADDR:
        // second least significant bit in PC is always zero
        ipr[idx] = val & ~2;
        break;

      case TheISA::IPR_ASTRR:
      case TheISA::IPR_ASTER:
        // only write least significant four bits - privilege mask
        ipr[idx] = val & 0xf;
        break;

      case TheISA::IPR_IPLR:
        // only write least significant five bits - interrupt level
        ipr[idx] = val & 0x1f;
        break;

      case TheISA::IPR_DTB_CM:

      case TheISA::IPR_ICM:
        // only write two mode bits - processor mode
        ipr[idx] = val & 0x18;
        break;

      case TheISA::IPR_ALT_MODE:
        // only write two mode bits - processor mode
        ipr[idx] = val & 0x18;
        break;

      case TheISA::IPR_MCSR:
        // more here after optimization...
        ipr[idx] = val;
        break;

      case TheISA::IPR_SIRR:
        // only write software interrupt mask
        ipr[idx] = val & 0x7fff0;
        break;

      case TheISA::IPR_ICSR:
        ipr[idx] = val & ULL(0xffffff0300);
        break;

      case TheISA::IPR_IVPTBR:
      case TheISA::IPR_MVPTBR:
        ipr[idx] = val & ULL(0xffffffffc0000000);
        break;

      case TheISA::IPR_DC_TEST_CTL:
        ipr[idx] = val & 0x1ffb;
        break;

      case TheISA::IPR_DC_MODE:
      case TheISA::IPR_MAF_MODE:
        ipr[idx] = val & 0x3f;
        break;

      case TheISA::IPR_ITB_ASN:
        ipr[idx] = val & 0x7f0;
        break;

      case TheISA::IPR_DTB_ASN:
        ipr[idx] = val & ULL(0xfe00000000000000);
        break;

      case TheISA::IPR_EXC_SUM:
      case TheISA::IPR_EXC_MASK:
        // any write to this register clears it
        ipr[idx] = 0;
        break;

      case TheISA::IPR_INTID:
      case TheISA::IPR_SL_RCV:
      case TheISA::IPR_MM_STAT:
      case TheISA::IPR_ITB_PTE_TEMP:
      case TheISA::IPR_DTB_PTE_TEMP:
        // read-only registers
        return UnimplementedOpcodeFault;

      case TheISA::IPR_HWINT_CLR:
      case TheISA::IPR_SL_XMIT:
      case TheISA::IPR_DC_FLUSH:
      case TheISA::IPR_IC_FLUSH:
        // the following are write only
        ipr[idx] = val;
        break;

      case TheISA::IPR_DTB_IA:
        // really a control write
        ipr[idx] = 0;

        cpu->dtb->flushAll();
        break;

      case TheISA::IPR_DTB_IAP:
        // really a control write
        ipr[idx] = 0;

        cpu->dtb->flushProcesses();
        break;

      case TheISA::IPR_DTB_IS:
        // really a control write
        ipr[idx] = val;

        cpu->dtb->flushAddr(val, DTB_ASN_ASN(ipr[TheISA::IPR_DTB_ASN]));
        break;

      case TheISA::IPR_DTB_TAG: {
          struct TheISA::PTE pte;

          // FIXME: granularity hints NYI...
          if (DTB_PTE_GH(ipr[TheISA::IPR_DTB_PTE]) != 0)
              panic("PTE GH field != 0");

          // write entire quad
          ipr[idx] = val;

          // construct PTE for new entry
          pte.ppn = DTB_PTE_PPN(ipr[TheISA::IPR_DTB_PTE]);
          pte.xre = DTB_PTE_XRE(ipr[TheISA::IPR_DTB_PTE]);
          pte.xwe = DTB_PTE_XWE(ipr[TheISA::IPR_DTB_PTE]);
          pte.fonr = DTB_PTE_FONR(ipr[TheISA::IPR_DTB_PTE]);
          pte.fonw = DTB_PTE_FONW(ipr[TheISA::IPR_DTB_PTE]);
          pte.asma = DTB_PTE_ASMA(ipr[TheISA::IPR_DTB_PTE]);
          pte.asn = DTB_ASN_ASN(ipr[TheISA::IPR_DTB_ASN]);

          // insert new TAG/PTE value into data TLB
          cpu->dtb->insert(val, pte);
      }
        break;

      case TheISA::IPR_ITB_PTE: {
          struct TheISA::PTE pte;

          // FIXME: granularity hints NYI...
          if (ITB_PTE_GH(val) != 0)
              panic("PTE GH field != 0");

          // write entire quad
          ipr[idx] = val;

          // construct PTE for new entry
          pte.ppn = ITB_PTE_PPN(val);
          pte.xre = ITB_PTE_XRE(val);
          pte.xwe = 0;
          pte.fonr = ITB_PTE_FONR(val);
          pte.fonw = ITB_PTE_FONW(val);
          pte.asma = ITB_PTE_ASMA(val);
          pte.asn = ITB_ASN_ASN(ipr[TheISA::IPR_ITB_ASN]);

          // insert new TAG/PTE value into data TLB
          cpu->itb->insert(ipr[TheISA::IPR_ITB_TAG], pte);
      }
        break;

      case TheISA::IPR_ITB_IA:
        // really a control write
        ipr[idx] = 0;

        cpu->itb->flushAll();
        break;

      case TheISA::IPR_ITB_IAP:
        // really a control write
        ipr[idx] = 0;

        cpu->itb->flushProcesses();
        break;

      case TheISA::IPR_ITB_IS:
        // really a control write
        ipr[idx] = val;

        cpu->itb->flushAddr(val, ITB_ASN_ASN(ipr[TheISA::IPR_ITB_ASN]));
        break;

      default:
        // invalid IPR
        return UnimplementedOpcodeFault;
    }

    // no error...
    return NoFault;
}

#endif // #if FULL_SYSTEM

#endif // __CPU_O3_CPU_REGFILE_HH__
