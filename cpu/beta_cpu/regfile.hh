#ifndef __CPU_BETA_CPU_REGFILE_HH__
#define __CPU_BETA_CPU_REGFILE_HH__

// @todo: Destructor

#include "arch/alpha/isa_traits.hh"
#include "base/trace.hh"
#include "cpu/beta_cpu/comm.hh"

#ifdef FULL_SYSTEM
#include "kern/kernel_stats.hh"
#include "arch/alpha/ev5.hh"

using namespace EV5;
#endif

// This really only depends on the ISA, and not the Impl.  It might be nicer
// to see if I can make it depend on nothing...
// Things that are in the ifdef FULL_SYSTEM are pretty dependent on the ISA,
// and should go in the AlphaFullCPU.

extern void debug_break();

template <class Impl>
class PhysRegFile
{
    //Note that most of the definitions of the IntReg, FloatReg, etc. exist
    //within the Impl/ISA class and not within this PhysRegFile class.

    //Will need some way to allow stuff like swap_palshadow to access the
    //correct registers.  Might require code changes to swap_palshadow and
    //other execution contexts.

    //Will make these registers public for now, but they probably should
    //be private eventually with some accessor functions.
  public:
    typedef typename Impl::ISA ISA;
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

#ifdef FULL_SYSTEM
    uint64_t readIpr(int idx, Fault &fault);
    Fault setIpr(int idx, uint64_t val);
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

    Addr pc;            // program counter
    Addr npc;            // next-cycle program counter

#ifdef FULL_SYSTEM
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

#ifdef FULL_SYSTEM

//Problem:  This code doesn't make sense at the RegFile level because it
//needs things such as the itb and dtb.  Either put it at the CPU level or
//the DynInst level.
template <class Impl>
uint64_t
PhysRegFile<Impl>::readIpr(int idx, Fault &fault)
{
    uint64_t retval = 0;    // return value, default 0

    switch (idx) {
      case ISA::IPR_PALtemp0:
      case ISA::IPR_PALtemp1:
      case ISA::IPR_PALtemp2:
      case ISA::IPR_PALtemp3:
      case ISA::IPR_PALtemp4:
      case ISA::IPR_PALtemp5:
      case ISA::IPR_PALtemp6:
      case ISA::IPR_PALtemp7:
      case ISA::IPR_PALtemp8:
      case ISA::IPR_PALtemp9:
      case ISA::IPR_PALtemp10:
      case ISA::IPR_PALtemp11:
      case ISA::IPR_PALtemp12:
      case ISA::IPR_PALtemp13:
      case ISA::IPR_PALtemp14:
      case ISA::IPR_PALtemp15:
      case ISA::IPR_PALtemp16:
      case ISA::IPR_PALtemp17:
      case ISA::IPR_PALtemp18:
      case ISA::IPR_PALtemp19:
      case ISA::IPR_PALtemp20:
      case ISA::IPR_PALtemp21:
      case ISA::IPR_PALtemp22:
      case ISA::IPR_PALtemp23:
      case ISA::IPR_PAL_BASE:

      case ISA::IPR_IVPTBR:
      case ISA::IPR_DC_MODE:
      case ISA::IPR_MAF_MODE:
      case ISA::IPR_ISR:
      case ISA::IPR_EXC_ADDR:
      case ISA::IPR_IC_PERR_STAT:
      case ISA::IPR_DC_PERR_STAT:
      case ISA::IPR_MCSR:
      case ISA::IPR_ASTRR:
      case ISA::IPR_ASTER:
      case ISA::IPR_SIRR:
      case ISA::IPR_ICSR:
      case ISA::IPR_ICM:
      case ISA::IPR_DTB_CM:
      case ISA::IPR_IPLR:
      case ISA::IPR_INTID:
      case ISA::IPR_PMCTR:
        // no side-effect
        retval = ipr[idx];
        break;

      case ISA::IPR_CC:
        retval |= ipr[idx] & ULL(0xffffffff00000000);
        retval |= curTick  & ULL(0x00000000ffffffff);
        break;

      case ISA::IPR_VA:
        retval = ipr[idx];
        break;

      case ISA::IPR_VA_FORM:
      case ISA::IPR_MM_STAT:
      case ISA::IPR_IFAULT_VA_FORM:
      case ISA::IPR_EXC_MASK:
      case ISA::IPR_EXC_SUM:
        retval = ipr[idx];
        break;

      case ISA::IPR_DTB_PTE:
        {
            typename ISA::PTE &pte = cpu->dtb->index(1);

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
      case ISA::IPR_HWINT_CLR:
      case ISA::IPR_SL_XMIT:
      case ISA::IPR_DC_FLUSH:
      case ISA::IPR_IC_FLUSH:
      case ISA::IPR_ALT_MODE:
      case ISA::IPR_DTB_IA:
      case ISA::IPR_DTB_IAP:
      case ISA::IPR_ITB_IA:
      case ISA::IPR_ITB_IAP:
        fault = Unimplemented_Opcode_Fault;
        break;

      default:
        // invalid IPR
        fault = Unimplemented_Opcode_Fault;
        break;
    }

    return retval;
}

extern int break_ipl;

template <class Impl>
Fault
PhysRegFile<Impl>::setIpr(int idx, uint64_t val)
{
    uint64_t old;

    switch (idx) {
      case ISA::IPR_PALtemp0:
      case ISA::IPR_PALtemp1:
      case ISA::IPR_PALtemp2:
      case ISA::IPR_PALtemp3:
      case ISA::IPR_PALtemp4:
      case ISA::IPR_PALtemp5:
      case ISA::IPR_PALtemp6:
      case ISA::IPR_PALtemp7:
      case ISA::IPR_PALtemp8:
      case ISA::IPR_PALtemp9:
      case ISA::IPR_PALtemp10:
      case ISA::IPR_PALtemp11:
      case ISA::IPR_PALtemp12:
      case ISA::IPR_PALtemp13:
      case ISA::IPR_PALtemp14:
      case ISA::IPR_PALtemp15:
      case ISA::IPR_PALtemp16:
      case ISA::IPR_PALtemp17:
      case ISA::IPR_PALtemp18:
      case ISA::IPR_PALtemp19:
      case ISA::IPR_PALtemp20:
      case ISA::IPR_PALtemp21:
      case ISA::IPR_PALtemp22:
      case ISA::IPR_PAL_BASE:
      case ISA::IPR_IC_PERR_STAT:
      case ISA::IPR_DC_PERR_STAT:
      case ISA::IPR_PMCTR:
        // write entire quad w/ no side-effect
        ipr[idx] = val;
        break;

      case ISA::IPR_CC_CTL:
        // This IPR resets the cycle counter.  We assume this only
        // happens once... let's verify that.
        assert(ipr[idx] == 0);
        ipr[idx] = 1;
        break;

      case ISA::IPR_CC:
        // This IPR only writes the upper 64 bits.  It's ok to write
        // all 64 here since we mask out the lower 32 in rpcc (see
        // isa_desc).
        ipr[idx] = val;
        break;

      case ISA::IPR_PALtemp23:
        // write entire quad w/ no side-effect
        old = ipr[idx];
        ipr[idx] = val;
//        kernelStats.context(old, val);
        break;

      case ISA::IPR_DTB_PTE:
        // write entire quad w/ no side-effect, tag is forthcoming
        ipr[idx] = val;
        break;

      case ISA::IPR_EXC_ADDR:
        // second least significant bit in PC is always zero
        ipr[idx] = val & ~2;
        break;

      case ISA::IPR_ASTRR:
      case ISA::IPR_ASTER:
        // only write least significant four bits - privilege mask
        ipr[idx] = val & 0xf;
        break;

      case ISA::IPR_IPLR:
#ifdef DEBUG
        if (break_ipl != -1 && break_ipl == (val & 0x1f))
            debug_break();
#endif

        // only write least significant five bits - interrupt level
        ipr[idx] = val & 0x1f;
//        kernelStats.swpipl(ipr[idx]);
        break;

      case ISA::IPR_DTB_CM:
//        if (val & 0x18)
//            kernelStats->mode(Kernel::user);
//        else
//            kernelStats->mode(Kernel::kernel);

      case ISA::IPR_ICM:
        // only write two mode bits - processor mode
        ipr[idx] = val & 0x18;
        break;

      case ISA::IPR_ALT_MODE:
        // only write two mode bits - processor mode
        ipr[idx] = val & 0x18;
        break;

      case ISA::IPR_MCSR:
        // more here after optimization...
        ipr[idx] = val;
        break;

      case ISA::IPR_SIRR:
        // only write software interrupt mask
        ipr[idx] = val & 0x7fff0;
        break;

      case ISA::IPR_ICSR:
        ipr[idx] = val & ULL(0xffffff0300);
        break;

      case ISA::IPR_IVPTBR:
      case ISA::IPR_MVPTBR:
        ipr[idx] = val & ULL(0xffffffffc0000000);
        break;

      case ISA::IPR_DC_TEST_CTL:
        ipr[idx] = val & 0x1ffb;
        break;

      case ISA::IPR_DC_MODE:
      case ISA::IPR_MAF_MODE:
        ipr[idx] = val & 0x3f;
        break;

      case ISA::IPR_ITB_ASN:
        ipr[idx] = val & 0x7f0;
        break;

      case ISA::IPR_DTB_ASN:
        ipr[idx] = val & ULL(0xfe00000000000000);
        break;

      case ISA::IPR_EXC_SUM:
      case ISA::IPR_EXC_MASK:
        // any write to this register clears it
        ipr[idx] = 0;
        break;

      case ISA::IPR_INTID:
      case ISA::IPR_SL_RCV:
      case ISA::IPR_MM_STAT:
      case ISA::IPR_ITB_PTE_TEMP:
      case ISA::IPR_DTB_PTE_TEMP:
        // read-only registers
        return Unimplemented_Opcode_Fault;

      case ISA::IPR_HWINT_CLR:
      case ISA::IPR_SL_XMIT:
      case ISA::IPR_DC_FLUSH:
      case ISA::IPR_IC_FLUSH:
        // the following are write only
        ipr[idx] = val;
        break;

      case ISA::IPR_DTB_IA:
        // really a control write
        ipr[idx] = 0;

        cpu->dtb->flushAll();
        break;

      case ISA::IPR_DTB_IAP:
        // really a control write
        ipr[idx] = 0;

        cpu->dtb->flushProcesses();
        break;

      case ISA::IPR_DTB_IS:
        // really a control write
        ipr[idx] = val;

        cpu->dtb->flushAddr(val, DTB_ASN_ASN(ipr[ISA::IPR_DTB_ASN]));
        break;

      case ISA::IPR_DTB_TAG: {
          struct ISA::PTE pte;

          // FIXME: granularity hints NYI...
          if (DTB_PTE_GH(ipr[ISA::IPR_DTB_PTE]) != 0)
              panic("PTE GH field != 0");

          // write entire quad
          ipr[idx] = val;

          // construct PTE for new entry
          pte.ppn = DTB_PTE_PPN(ipr[ISA::IPR_DTB_PTE]);
          pte.xre = DTB_PTE_XRE(ipr[ISA::IPR_DTB_PTE]);
          pte.xwe = DTB_PTE_XWE(ipr[ISA::IPR_DTB_PTE]);
          pte.fonr = DTB_PTE_FONR(ipr[ISA::IPR_DTB_PTE]);
          pte.fonw = DTB_PTE_FONW(ipr[ISA::IPR_DTB_PTE]);
          pte.asma = DTB_PTE_ASMA(ipr[ISA::IPR_DTB_PTE]);
          pte.asn = DTB_ASN_ASN(ipr[ISA::IPR_DTB_ASN]);

          // insert new TAG/PTE value into data TLB
          cpu->dtb->insert(val, pte);
      }
        break;

      case ISA::IPR_ITB_PTE: {
          struct ISA::PTE pte;

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
          pte.asn = ITB_ASN_ASN(ipr[ISA::IPR_ITB_ASN]);

          // insert new TAG/PTE value into data TLB
          cpu->itb->insert(ipr[ISA::IPR_ITB_TAG], pte);
      }
        break;

      case ISA::IPR_ITB_IA:
        // really a control write
        ipr[idx] = 0;

        cpu->itb->flushAll();
        break;

      case ISA::IPR_ITB_IAP:
        // really a control write
        ipr[idx] = 0;

        cpu->itb->flushProcesses();
        break;

      case ISA::IPR_ITB_IS:
        // really a control write
        ipr[idx] = val;

        cpu->itb->flushAddr(val, ITB_ASN_ASN(ipr[ISA::IPR_ITB_ASN]));
        break;

      default:
        // invalid IPR
        return Unimplemented_Opcode_Fault;
    }

    // no error...
    return No_Fault;
}

#endif // #ifdef FULL_SYSTEM

#endif // __CPU_BETA_CPU_REGFILE_HH__
