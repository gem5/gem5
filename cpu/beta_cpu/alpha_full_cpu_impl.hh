
#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "mem/cache/cache.hh" // for dynamic cast
#include "mem/mem_interface.hh"
#include "sim/builder.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"

#include "cpu/beta_cpu/alpha_full_cpu.hh"
#include "cpu/beta_cpu/alpha_params.hh"
#include "cpu/beta_cpu/comm.hh"

template <class Impl>
AlphaFullCPU<Impl>::AlphaFullCPU(Params &params)
    : FullBetaCPU<AlphaSimpleImpl>(params)
{
    DPRINTF(FullCPU, "AlphaFullCPU: Creating AlphaFullCPU object.\n");

    fetch.setCPU(this);
    decode.setCPU(this);
    rename.setCPU(this);
    iew.setCPU(this);
    commit.setCPU(this);

    rob.setCPU(this);
}

template <class Impl>
void
AlphaFullCPU<Impl>::regStats()
{
    // Register stats for everything that has stats.
    fullCPURegStats();
    fetch.regStats();
    decode.regStats();
    rename.regStats();
    iew.regStats();
    commit.regStats();
}

#ifndef FULL_SYSTEM

template <class Impl>
void
AlphaFullCPU<Impl>::syscall()
{
    DPRINTF(FullCPU, "AlphaFullCPU: Syscall() called.\n\n");

    // Commit stage needs to run as well.
    commit.tick();

    squashStages();

    // Temporarily increase this by one to account for the syscall
    // instruction.
    ++funcExeInst;

    // Copy over all important state to xc once all the unrolling is done.
    copyToXC();

    process->syscall(xc);

    // Copy over all important state back to CPU.
    copyFromXC();

    // Decrease funcExeInst by one as the normal commit will handle
    // incrememnting it.
    --funcExeInst;
}

// This is not a pretty function, and should only be used if it is necessary
// to fake having everything squash all at once (ie for non-full system
// syscalls).  Maybe put this at the FullCPU level?
template <class Impl>
void
AlphaFullCPU<Impl>::squashStages()
{
    InstSeqNum rob_head = rob.readHeadSeqNum();

    // Now hack the time buffer to put this sequence number in the places
    // where the stages might read it.
    for (int i = 0; i < 5; ++i)
    {
        timeBuffer.access(-i)->commitInfo.doneSeqNum = rob_head;
    }

    fetch.squash(rob.readHeadNextPC());
    fetchQueue.advance();

    decode.squash();
    decodeQueue.advance();

    rename.squash();
    renameQueue.advance();
    renameQueue.advance();

    // Be sure to advance the IEW queues so that the commit stage doesn't
    // try to set an instruction as completed at the same time that it
    // might be deleting it.
    iew.squash();
    iewQueue.advance();
    iewQueue.advance();

    rob.squash(rob_head);
    commit.setSquashing();

    // Now hack the time buffer to clear the sequence numbers in the places
    // where the stages might read it.?
    for (int i = 0; i < 5; ++i)
    {
        timeBuffer.access(-i)->commitInfo.doneSeqNum = 0;
    }

}

#endif // FULL_SYSTEM

template <class Impl>
void
AlphaFullCPU<Impl>::copyToXC()
{
    PhysRegIndex renamed_reg;

    // First loop through the integer registers.
    for (int i = 0; i < AlphaISA::NumIntRegs; ++i)
    {
        renamed_reg = renameMap.lookup(i);
        xc->regs.intRegFile[i] = regFile.readIntReg(renamed_reg);
        DPRINTF(FullCPU, "FullCPU: Copying register %i, has data %lli.\n",
                renamed_reg, regFile.intRegFile[renamed_reg]);
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < AlphaISA::NumFloatRegs; ++i)
    {
        renamed_reg = renameMap.lookup(i + AlphaISA::FP_Base_DepTag);
        xc->regs.floatRegFile.d[i] = regFile.readFloatRegDouble(renamed_reg);
        xc->regs.floatRegFile.q[i] = regFile.readFloatRegInt(renamed_reg);
    }

    xc->regs.miscRegs.fpcr = regFile.miscRegs.fpcr;
    xc->regs.miscRegs.uniq = regFile.miscRegs.uniq;
    xc->regs.miscRegs.lock_flag = regFile.miscRegs.lock_flag;
    xc->regs.miscRegs.lock_addr = regFile.miscRegs.lock_addr;

    xc->regs.pc = rob.readHeadPC();
    xc->regs.npc = xc->regs.pc+4;

    xc->func_exe_inst = funcExeInst;
}

// This function will probably mess things up unless the ROB is empty and
// there are no instructions in the pipeline.
template <class Impl>
void
AlphaFullCPU<Impl>::copyFromXC()
{
    PhysRegIndex renamed_reg;

    // First loop through the integer registers.
    for (int i = 0; i < AlphaISA::NumIntRegs; ++i)
    {
        renamed_reg = renameMap.lookup(i);

        DPRINTF(FullCPU, "FullCPU: Copying over register %i, had data %lli, "
                "now has data %lli.\n",
                renamed_reg, regFile.intRegFile[renamed_reg],
                xc->regs.intRegFile[i]);

        regFile.setIntReg(renamed_reg, xc->regs.intRegFile[i]);
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < AlphaISA::NumFloatRegs; ++i)
    {
        renamed_reg = renameMap.lookup(i + AlphaISA::FP_Base_DepTag);
        regFile.setFloatRegDouble(renamed_reg, xc->regs.floatRegFile.d[i]);
        regFile.setFloatRegInt(renamed_reg, xc->regs.floatRegFile.q[i]);
    }

    // Then loop through the misc registers.
    regFile.miscRegs.fpcr = xc->regs.miscRegs.fpcr;
    regFile.miscRegs.uniq = xc->regs.miscRegs.uniq;
    regFile.miscRegs.lock_flag = xc->regs.miscRegs.lock_flag;
    regFile.miscRegs.lock_addr = xc->regs.miscRegs.lock_addr;

    // Then finally set the PC and the next PC.
//    regFile.pc = xc->regs.pc;
//    regFile.npc = xc->regs.npc;

    funcExeInst = xc->func_exe_inst;
}

#ifdef FULL_SYSTEM

template <class Impl>
uint64_t *
AlphaFullCPU<Impl>::getIpr()
{
    return regFile.getIpr();
}

template <class Impl>
uint64_t
AlphaFullCPU<Impl>::readIpr(int idx, Fault &fault)
{
    uint64_t *ipr = getIpr();
    uint64_t retval = 0;	// return value, default 0

    switch (idx) {
      case AlphaISA::IPR_PALtemp0:
      case AlphaISA::IPR_PALtemp1:
      case AlphaISA::IPR_PALtemp2:
      case AlphaISA::IPR_PALtemp3:
      case AlphaISA::IPR_PALtemp4:
      case AlphaISA::IPR_PALtemp5:
      case AlphaISA::IPR_PALtemp6:
      case AlphaISA::IPR_PALtemp7:
      case AlphaISA::IPR_PALtemp8:
      case AlphaISA::IPR_PALtemp9:
      case AlphaISA::IPR_PALtemp10:
      case AlphaISA::IPR_PALtemp11:
      case AlphaISA::IPR_PALtemp12:
      case AlphaISA::IPR_PALtemp13:
      case AlphaISA::IPR_PALtemp14:
      case AlphaISA::IPR_PALtemp15:
      case AlphaISA::IPR_PALtemp16:
      case AlphaISA::IPR_PALtemp17:
      case AlphaISA::IPR_PALtemp18:
      case AlphaISA::IPR_PALtemp19:
      case AlphaISA::IPR_PALtemp20:
      case AlphaISA::IPR_PALtemp21:
      case AlphaISA::IPR_PALtemp22:
      case AlphaISA::IPR_PALtemp23:
      case AlphaISA::IPR_PAL_BASE:

      case AlphaISA::IPR_IVPTBR:
      case AlphaISA::IPR_DC_MODE:
      case AlphaISA::IPR_MAF_MODE:
      case AlphaISA::IPR_ISR:
      case AlphaISA::IPR_EXC_ADDR:
      case AlphaISA::IPR_IC_PERR_STAT:
      case AlphaISA::IPR_DC_PERR_STAT:
      case AlphaISA::IPR_MCSR:
      case AlphaISA::IPR_ASTRR:
      case AlphaISA::IPR_ASTER:
      case AlphaISA::IPR_SIRR:
      case AlphaISA::IPR_ICSR:
      case AlphaISA::IPR_ICM:
      case AlphaISA::IPR_DTB_CM:
      case AlphaISA::IPR_IPLR:
      case AlphaISA::IPR_INTID:
      case AlphaISA::IPR_PMCTR:
        // no side-effect
        retval = ipr[idx];
        break;

      case AlphaISA::IPR_CC:
        retval |= ipr[idx] & ULL(0xffffffff00000000);
        retval |= curTick  & ULL(0x00000000ffffffff);
        break;

      case AlphaISA::IPR_VA:
        retval = ipr[idx];
        break;

      case AlphaISA::IPR_VA_FORM:
      case AlphaISA::IPR_MM_STAT:
      case AlphaISA::IPR_IFAULT_VA_FORM:
      case AlphaISA::IPR_EXC_MASK:
      case AlphaISA::IPR_EXC_SUM:
        retval = ipr[idx];
        break;

      case AlphaISA::IPR_DTB_PTE:
        {
            AlphaISA::PTE &pte = dtb->index(!misspeculating());

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
      case AlphaISA::IPR_HWINT_CLR:
      case AlphaISA::IPR_SL_XMIT:
      case AlphaISA::IPR_DC_FLUSH:
      case AlphaISA::IPR_IC_FLUSH:
      case AlphaISA::IPR_ALT_MODE:
      case AlphaISA::IPR_DTB_IA:
      case AlphaISA::IPR_DTB_IAP:
      case AlphaISA::IPR_ITB_IA:
      case AlphaISA::IPR_ITB_IAP:
        fault = Unimplemented_Opcode_Fault;
        break;

      default:
        // invalid IPR
        fault = Unimplemented_Opcode_Fault;
        break;
    }

    return retval;
}

template <class Impl>
Fault
AlphaFullCPU<Impl>::setIpr(int idx, uint64_t val)
{
    uint64_t *ipr = getIpr();
    uint64_t old;

    if (misspeculating())
        return No_Fault;

    switch (idx) {
      case AlphaISA::IPR_PALtemp0:
      case AlphaISA::IPR_PALtemp1:
      case AlphaISA::IPR_PALtemp2:
      case AlphaISA::IPR_PALtemp3:
      case AlphaISA::IPR_PALtemp4:
      case AlphaISA::IPR_PALtemp5:
      case AlphaISA::IPR_PALtemp6:
      case AlphaISA::IPR_PALtemp7:
      case AlphaISA::IPR_PALtemp8:
      case AlphaISA::IPR_PALtemp9:
      case AlphaISA::IPR_PALtemp10:
      case AlphaISA::IPR_PALtemp11:
      case AlphaISA::IPR_PALtemp12:
      case AlphaISA::IPR_PALtemp13:
      case AlphaISA::IPR_PALtemp14:
      case AlphaISA::IPR_PALtemp15:
      case AlphaISA::IPR_PALtemp16:
      case AlphaISA::IPR_PALtemp17:
      case AlphaISA::IPR_PALtemp18:
      case AlphaISA::IPR_PALtemp19:
      case AlphaISA::IPR_PALtemp20:
      case AlphaISA::IPR_PALtemp21:
      case AlphaISA::IPR_PALtemp22:
      case AlphaISA::IPR_PAL_BASE:
      case AlphaISA::IPR_IC_PERR_STAT:
      case AlphaISA::IPR_DC_PERR_STAT:
      case AlphaISA::IPR_PMCTR:
        // write entire quad w/ no side-effect
        ipr[idx] = val;
        break;

      case AlphaISA::IPR_CC_CTL:
        // This IPR resets the cycle counter.  We assume this only
        // happens once... let's verify that.
        assert(ipr[idx] == 0);
        ipr[idx] = 1;
        break;

      case AlphaISA::IPR_CC:
        // This IPR only writes the upper 64 bits.  It's ok to write
        // all 64 here since we mask out the lower 32 in rpcc (see
        // isa_desc).
        ipr[idx] = val;
        break;

      case AlphaISA::IPR_PALtemp23:
        // write entire quad w/ no side-effect
        old = ipr[idx];
        ipr[idx] = val;
        kernelStats.context(old, val);
        break;

      case AlphaISA::IPR_DTB_PTE:
        // write entire quad w/ no side-effect, tag is forthcoming
        ipr[idx] = val;
        break;

      case AlphaISA::IPR_EXC_ADDR:
        // second least significant bit in PC is always zero
        ipr[idx] = val & ~2;
        break;

      case AlphaISA::IPR_ASTRR:
      case AlphaISA::IPR_ASTER:
        // only write least significant four bits - privilege mask
        ipr[idx] = val & 0xf;
        break;

      case AlphaISA::IPR_IPLR:
#ifdef DEBUG
        if (break_ipl != -1 && break_ipl == (val & 0x1f))
            debug_break();
#endif

        // only write least significant five bits - interrupt level
        ipr[idx] = val & 0x1f;
        kernelStats.swpipl(ipr[idx]);
        break;

      case AlphaISA::IPR_DTB_CM:
        kernelStats.mode((val & 0x18) != 0);

      case AlphaISA::IPR_ICM:
        // only write two mode bits - processor mode
        ipr[idx] = val & 0x18;
        break;

      case AlphaISA::IPR_ALT_MODE:
        // only write two mode bits - processor mode
        ipr[idx] = val & 0x18;
        break;

      case AlphaISA::IPR_MCSR:
        // more here after optimization...
        ipr[idx] = val;
        break;

      case AlphaISA::IPR_SIRR:
        // only write software interrupt mask
        ipr[idx] = val & 0x7fff0;
        break;

      case AlphaISA::IPR_ICSR:
        ipr[idx] = val & ULL(0xffffff0300);
        break;

      case AlphaISA::IPR_IVPTBR:
      case AlphaISA::IPR_MVPTBR:
        ipr[idx] = val & ULL(0xffffffffc0000000);
        break;

      case AlphaISA::IPR_DC_TEST_CTL:
        ipr[idx] = val & 0x1ffb;
        break;

      case AlphaISA::IPR_DC_MODE:
      case AlphaISA::IPR_MAF_MODE:
        ipr[idx] = val & 0x3f;
        break;

      case AlphaISA::IPR_ITB_ASN:
        ipr[idx] = val & 0x7f0;
        break;

      case AlphaISA::IPR_DTB_ASN:
        ipr[idx] = val & ULL(0xfe00000000000000);
        break;

      case AlphaISA::IPR_EXC_SUM:
      case AlphaISA::IPR_EXC_MASK:
        // any write to this register clears it
        ipr[idx] = 0;
        break;

      case AlphaISA::IPR_INTID:
      case AlphaISA::IPR_SL_RCV:
      case AlphaISA::IPR_MM_STAT:
      case AlphaISA::IPR_ITB_PTE_TEMP:
      case AlphaISA::IPR_DTB_PTE_TEMP:
        // read-only registers
        return Unimplemented_Opcode_Fault;

      case AlphaISA::IPR_HWINT_CLR:
      case AlphaISA::IPR_SL_XMIT:
      case AlphaISA::IPR_DC_FLUSH:
      case AlphaISA::IPR_IC_FLUSH:
        // the following are write only
        ipr[idx] = val;
        break;

      case AlphaISA::IPR_DTB_IA:
        // really a control write
        ipr[idx] = 0;

        dtb->flushAll();
        break;

      case AlphaISA::IPR_DTB_IAP:
        // really a control write
        ipr[idx] = 0;

        dtb->flushProcesses();
        break;

      case AlphaISA::IPR_DTB_IS:
        // really a control write
        ipr[idx] = val;

        dtb->flushAddr(val, DTB_ASN_ASN(ipr[AlphaISA::IPR_DTB_ASN]));
        break;

      case AlphaISA::IPR_DTB_TAG: {
          struct AlphaISA::PTE pte;

          // FIXME: granularity hints NYI...
          if (DTB_PTE_GH(ipr[AlphaISA::IPR_DTB_PTE]) != 0)
              panic("PTE GH field != 0");

          // write entire quad
          ipr[idx] = val;

          // construct PTE for new entry
          pte.ppn = DTB_PTE_PPN(ipr[AlphaISA::IPR_DTB_PTE]);
          pte.xre = DTB_PTE_XRE(ipr[AlphaISA::IPR_DTB_PTE]);
          pte.xwe = DTB_PTE_XWE(ipr[AlphaISA::IPR_DTB_PTE]);
          pte.fonr = DTB_PTE_FONR(ipr[AlphaISA::IPR_DTB_PTE]);
          pte.fonw = DTB_PTE_FONW(ipr[AlphaISA::IPR_DTB_PTE]);
          pte.asma = DTB_PTE_ASMA(ipr[AlphaISA::IPR_DTB_PTE]);
          pte.asn = DTB_ASN_ASN(ipr[AlphaISA::IPR_DTB_ASN]);

          // insert new TAG/PTE value into data TLB
          dtb->insert(val, pte);
      }
        break;

      case AlphaISA::IPR_ITB_PTE: {
          struct AlphaISA::PTE pte;

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
          pte.asn = ITB_ASN_ASN(ipr[AlphaISA::IPR_ITB_ASN]);

          // insert new TAG/PTE value into data TLB
          itb->insert(ipr[AlphaISA::IPR_ITB_TAG], pte);
      }
        break;

      case AlphaISA::IPR_ITB_IA:
        // really a control write
        ipr[idx] = 0;

        itb->flushAll();
        break;

      case AlphaISA::IPR_ITB_IAP:
        // really a control write
        ipr[idx] = 0;

        itb->flushProcesses();
        break;

      case AlphaISA::IPR_ITB_IS:
        // really a control write
        ipr[idx] = val;

        itb->flushAddr(val, ITB_ASN_ASN(ipr[AlphaISA::IPR_ITB_ASN]));
        break;

      default:
        // invalid IPR
        return Unimplemented_Opcode_Fault;
    }

    // no error...
    return No_Fault;

}

template <class Impl>
int
AlphaFullCPU<Impl>::readIntrFlag()
{
    return regs.intrflag;
}

template <class Impl>
void
AlphaFullCPU<Impl>::setIntrFlag(int val)
{
    regs.intrflag = val;
}

// Can force commit stage to squash and stuff.
template <class Impl>
Fault
AlphaFullCPU<Impl>::hwrei()
{
    uint64_t *ipr = getIpr();

    if (!PC_PAL(regs.pc))
        return Unimplemented_Opcode_Fault;

    setNextPC(ipr[AlphaISA::IPR_EXC_ADDR]);

    if (!misspeculating()) {
        kernelStats.hwrei();

        if ((ipr[AlphaISA::IPR_EXC_ADDR] & 1) == 0)
            AlphaISA::swap_palshadow(&regs, false);

        AlphaISA::check_interrupts = true;
    }

    // FIXME: XXX check for interrupts? XXX
    return No_Fault;
}

template <class Impl>
bool
AlphaFullCPU<Impl>::inPalMode()
{
    return PC_PAL(readPC());
}

template <class Impl>
bool
AlphaFullCPU<Impl>::simPalCheck(int palFunc)
{
    kernelStats.callpal(palFunc);

    switch (palFunc) {
      case PAL::halt:
        halt();
        if (--System::numSystemsRunning == 0)
            new SimExitEvent("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (system->breakpoint())
            return false;
        break;
    }

    return true;
}

// Probably shouldn't be able to switch to the trap handler as quickly as
// this.  Also needs to get the exception restart address from the commit
// stage.
template <class Impl>
void
AlphaFullCPU<Impl>::trap(Fault fault)
{
    uint64_t PC = commit.readPC();

    DPRINTF(Fault, "Fault %s\n", FaultName(fault));
    Stats::recordEvent(csprintf("Fault %s", FaultName(fault)));

    assert(!misspeculating());
    kernelStats.fault(fault);

    if (fault == Arithmetic_Fault)
        panic("Arithmetic traps are unimplemented!");

    AlphaISA::InternalProcReg *ipr = getIpr();

    // exception restart address - Get the commit PC
    if (fault != Interrupt_Fault || !PC_PAL(PC))
        ipr[AlphaISA::IPR_EXC_ADDR] = PC;

    if (fault == Pal_Fault || fault == Arithmetic_Fault /* ||
        fault == Interrupt_Fault && !PC_PAL(regs.pc) */) {
        // traps...  skip faulting instruction
        ipr[AlphaISA::IPR_EXC_ADDR] += 4;
    }

    if (!PC_PAL(PC))
        AlphaISA::swap_palshadow(&regs, true);

    setPC( ipr[AlphaISA::IPR_PAL_BASE] + AlphaISA::fault_addr[fault] );
    setNextPC(PC + sizeof(MachInst));
}

template <class Impl>
void
AlphaFullCPU<Impl>::processInterrupts()
{
    // Check for interrupts here.  For now can copy the code that exists
    // within isa_fullsys_traits.hh.
}

// swap_palshadow swaps in the values of the shadow registers and
// swaps them with the values of the physical registers that map to the
// same logical index.
template <class Impl>
void
AlphaFullCPU<Impl>::swap_palshadow(RegFile *regs, bool use_shadow)
{
    if (palShadowEnabled == use_shadow)
        panic("swap_palshadow: wrong PAL shadow state");

    palShadowEnabled = use_shadow;

    // Will have to lookup in rename map to get physical registers, then
    // swap.
    for (int i = 0; i < AlphaISA::NumIntRegs; i++) {
        if (reg_redir[i]) {
            AlphaISA::IntReg temp = regs->intRegFile[i];
            regs->intRegFile[i] = regs->palregs[i];
            regs->palregs[i] = temp;
        }
    }
}

#endif // FULL_SYSTEM
