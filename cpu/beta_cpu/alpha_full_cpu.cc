
#include "base/cprintf.hh"
#include "base/statistics.hh"
#include "base/timebuf.hh"
#include "cpu/full_cpu/dd_queue.hh"
#include "cpu/full_cpu/full_cpu.hh"
#include "cpu/full_cpu/rob_station.hh"
#include "mem/cache/cache.hh" // for dynamic cast
#include "mem/mem_interface.hh"
#include "sim/builder.hh"
#include "sim/sim_events.hh"
#include "sim/stats.hh"

#include "cpu/beta_cpu/alpha_full_cpu.hh"
#include "cpu/beta_cpu/alpha_params.hh"
#include "cpu/beta_cpu/comm.hh"

AlphaFullCPU::AlphaFullCPU(Params &params)
    : FullBetaCPU<AlphaSimpleImpl>(params)
{

    fetch.setCPU(this);
    decode.setCPU(this);
    rename.setCPU(this);
    iew.setCPU(this);
    commit.setCPU(this);

    rob.setCPU(this);
}

#ifndef FULL_SYSTEM

void
AlphaFullCPU::syscall()
{
    DPRINTF(FullCPU, "AlphaFullCPU: Syscall() called.\n\n");

    squashStages();

    // Copy over all important state to xc once all the unrolling is done.
    copyToXC();

    process->syscall(xc);

    // Copy over all important state back to normal.
    copyFromXC();
}

// This is not a pretty function, and should only be used if it is necessary
// to fake having everything squash all at once (ie for non-full system
// syscalls).
void
AlphaFullCPU::squashStages()
{
    InstSeqNum rob_head = rob.readHeadSeqNum();

    // Now hack the time buffer to put this sequence number in the places
    // where the stages might read it.
    for (int i = 0; i < 10; ++i)
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

    iew.squash();
    iewQueue.advance();
    iewQueue.advance();

    rob.squash(rob_head);
    commit.setSquashing();
}

#endif // FULL_SYSTEM

void
AlphaFullCPU::copyToXC()
{
    PhysRegIndex renamed_reg;

    // First loop through the integer registers.
    for (int i = 0; i < AlphaISA::NumIntRegs; ++i)
    {
        renamed_reg = renameMap.lookup(i);
        xc->regs.intRegFile[i] = regFile.intRegFile[renamed_reg];
        DPRINTF(FullCPU, "FullCPU: Copying register %i, has data %lli.\n",
                renamed_reg, regFile.intRegFile[renamed_reg]);
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < AlphaISA::NumFloatRegs; ++i)
    {
        renamed_reg = renameMap.lookup(i + AlphaISA::FP_Base_DepTag);
        xc->regs.floatRegFile.d[i] = regFile.floatRegFile[renamed_reg].d;
        xc->regs.floatRegFile.q[i] = regFile.floatRegFile[renamed_reg].q;
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
void
AlphaFullCPU::copyFromXC()
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

        regFile.intRegFile[renamed_reg] = xc->regs.intRegFile[i];
    }

    // Then loop through the floating point registers.
    for (int i = 0; i < AlphaISA::NumFloatRegs; ++i)
    {
        renamed_reg = renameMap.lookup(i + AlphaISA::FP_Base_DepTag);
        regFile.floatRegFile[renamed_reg].d = xc->regs.floatRegFile.d[i];
        regFile.floatRegFile[renamed_reg].q = xc->regs.floatRegFile.q[i] ;
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

uint64_t *
AlphaFullCPU::getIpr()
{
    return regs.ipr;
}

uint64_t
AlphaFullCPU::readIpr(int idx, Fault &fault)
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

Fault
AlphaFullCPU::setIpr(int idx, uint64_t val)
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

int
AlphaFullCPU::readIntrFlag()
{
    return regs.intrflag;
}

void
AlphaFullCPU::setIntrFlag(int val)
{
    regs.intrflag = val;
}

// Maybe have this send back from IEW stage to squash and update PC.
Fault
AlphaFullCPU::hwrei()
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

bool
AlphaFullCPU::inPalMode()
{
    return PC_PAL(readPC());
}

bool
AlphaFullCPU::simPalCheck(int palFunc)
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
void
AlphaFullCPU::trap(Fault fault)
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

void
AlphaFullCPU::processInterrupts()
{
    // Check for interrupts here.  For now can copy the code that exists
    // within isa_fullsys_traits.hh.
}

// swap_palshadow swaps in the values of the shadow registers and
// swaps them with the values of the physical registers that map to the
// same logical index.
void
AlphaFullCPU::swap_palshadow(RegFile *regs, bool use_shadow)
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

BEGIN_DECLARE_SIM_OBJECT_PARAMS(AlphaFullCPU)

    Param<int> numThreads;

#ifdef FULL_SYSTEM
SimObjectParam<System *> system;
SimObjectParam<AlphaITB *> itb;
SimObjectParam<AlphaDTB *> dtb;
Param<int> mult;
#else
SimObjectVectorParam<Process *> workload;
SimObjectParam<Process *> process;
Param<short> asid;
#endif // FULL_SYSTEM
SimObjectParam<FunctionalMemory *> mem;

Param<Counter> max_insts_any_thread;
Param<Counter> max_insts_all_threads;
Param<Counter> max_loads_any_thread;
Param<Counter> max_loads_all_threads;

SimObjectParam<BaseCache *> icache;
SimObjectParam<BaseCache *> dcache;

Param<unsigned> decodeToFetchDelay;
Param<unsigned> renameToFetchDelay;
Param<unsigned> iewToFetchDelay;
Param<unsigned> commitToFetchDelay;
Param<unsigned> fetchWidth;

Param<unsigned> renameToDecodeDelay;
Param<unsigned> iewToDecodeDelay;
Param<unsigned> commitToDecodeDelay;
Param<unsigned> fetchToDecodeDelay;
Param<unsigned> decodeWidth;

Param<unsigned> iewToRenameDelay;
Param<unsigned> commitToRenameDelay;
Param<unsigned> decodeToRenameDelay;
Param<unsigned> renameWidth;

Param<unsigned> commitToIEWDelay;
Param<unsigned> renameToIEWDelay;
Param<unsigned> issueToExecuteDelay;
Param<unsigned> issueWidth;
Param<unsigned> executeWidth;
Param<unsigned> executeIntWidth;
Param<unsigned> executeFloatWidth;

Param<unsigned> iewToCommitDelay;
Param<unsigned> renameToROBDelay;
Param<unsigned> commitWidth;
Param<unsigned> squashWidth;

Param<unsigned> numPhysIntRegs;
Param<unsigned> numPhysFloatRegs;
Param<unsigned> numIQEntries;
Param<unsigned> numROBEntries;

Param<bool> defReg;

END_DECLARE_SIM_OBJECT_PARAMS(AlphaFullCPU)

BEGIN_INIT_SIM_OBJECT_PARAMS(AlphaFullCPU)

    INIT_PARAM(numThreads, "number of HW thread contexts"),

#ifdef FULL_SYSTEM
    INIT_PARAM(system, "System object"),
    INIT_PARAM(itb, "Instruction translation buffer"),
    INIT_PARAM(dtb, "Data translation buffer"),
    INIT_PARAM_DFLT(mult, "System clock multiplier", 1),
#else
    INIT_PARAM(workload, "Processes to run"),
    INIT_PARAM_DFLT(process, "Process to run", NULL),
    INIT_PARAM(asid, "Address space ID"),
#endif // FULL_SYSTEM

    INIT_PARAM_DFLT(mem, "Memory", NULL),

    INIT_PARAM_DFLT(max_insts_any_thread,
                    "Terminate when any thread reaches this inst count",
                    0),
    INIT_PARAM_DFLT(max_insts_all_threads,
                    "Terminate when all threads have reached"
                    "this inst count",
                    0),
    INIT_PARAM_DFLT(max_loads_any_thread,
                    "Terminate when any thread reaches this load count",
                    0),
    INIT_PARAM_DFLT(max_loads_all_threads,
                    "Terminate when all threads have reached this load"
                    "count",
                    0),

    INIT_PARAM_DFLT(icache, "L1 instruction cache", NULL),
    INIT_PARAM_DFLT(dcache, "L1 data cache", NULL),

    INIT_PARAM(decodeToFetchDelay, "Decode to fetch delay"),
    INIT_PARAM(renameToFetchDelay, "Rename to fetch delay"),
    INIT_PARAM(iewToFetchDelay, "Issue/Execute/Writeback to fetch"
               "delay"),
    INIT_PARAM(commitToFetchDelay, "Commit to fetch delay"),
    INIT_PARAM(fetchWidth, "Fetch width"),

    INIT_PARAM(renameToDecodeDelay, "Rename to decode delay"),
    INIT_PARAM(iewToDecodeDelay, "Issue/Execute/Writeback to decode"
               "delay"),
    INIT_PARAM(commitToDecodeDelay, "Commit to decode delay"),
    INIT_PARAM(fetchToDecodeDelay, "Fetch to decode delay"),
    INIT_PARAM(decodeWidth, "Decode width"),

    INIT_PARAM(iewToRenameDelay, "Issue/Execute/Writeback to rename"
               "delay"),
    INIT_PARAM(commitToRenameDelay, "Commit to rename delay"),
    INIT_PARAM(decodeToRenameDelay, "Decode to rename delay"),
    INIT_PARAM(renameWidth, "Rename width"),

    INIT_PARAM(commitToIEWDelay, "Commit to "
               "Issue/Execute/Writeback delay"),
    INIT_PARAM(renameToIEWDelay, "Rename to "
               "Issue/Execute/Writeback delay"),
    INIT_PARAM(issueToExecuteDelay, "Issue to execute delay (internal"
               "to the IEW stage)"),
    INIT_PARAM(issueWidth, "Issue width"),
    INIT_PARAM(executeWidth, "Execute width"),
    INIT_PARAM(executeIntWidth, "Integer execute width"),
    INIT_PARAM(executeFloatWidth, "Floating point execute width"),

    INIT_PARAM(iewToCommitDelay, "Issue/Execute/Writeback to commit "
               "delay"),
    INIT_PARAM(renameToROBDelay, "Rename to reorder buffer delay"),
    INIT_PARAM(commitWidth, "Commit width"),
    INIT_PARAM(squashWidth, "Squash width"),

    INIT_PARAM(numPhysIntRegs, "Number of physical integer registers"),
    INIT_PARAM(numPhysFloatRegs, "Number of physical floating point "
               "registers"),
    INIT_PARAM(numIQEntries, "Number of instruction queue entries"),
    INIT_PARAM(numROBEntries, "Number of reorder buffer entries"),

    INIT_PARAM(defReg, "Defer registration")

END_INIT_SIM_OBJECT_PARAMS(AlphaFullCPU)

CREATE_SIM_OBJECT(AlphaFullCPU)
{
    AlphaFullCPU *cpu;

#ifdef FULL_SYSTEM
    if (mult != 1)
        panic("Processor clock multiplier must be 1?\n");

    // Full-system only supports a single thread for the moment.
    int actual_num_threads = 1;
#else
    // In non-full-system mode, we infer the number of threads from
    // the workload if it's not explicitly specified.
    int actual_num_threads =
        numThreads.isValid() ? numThreads : workload.size();

    if (workload.size() == 0) {
        fatal("Must specify at least one workload!");
    }

    Process *actual_process;

    if (process == NULL) {
        actual_process = workload[0];
    } else {
        actual_process = process;
    }

#endif

    AlphaSimpleParams params;

    params.name = getInstanceName();
    params.numberOfThreads = actual_num_threads;

#ifdef FULL_SYSTEM
    params._system = system;
    params.itb = itb;
    params.dtb = dtb;
    params.freq = ticksPerSecond * mult;
#else
    params.workload = workload;
    params.process = actual_process;
    params.asid = asid;
#endif // FULL_SYSTEM

    params.mem = mem;

    params.maxInstsAnyThread = max_insts_any_thread;
    params.maxInstsAllThreads = max_insts_all_threads;
    params.maxLoadsAnyThread = max_loads_any_thread;
    params.maxLoadsAllThreads = max_loads_all_threads;

    //
    // Caches
    //
    params.icacheInterface = icache ? icache->getInterface() : NULL;
    params.dcacheInterface = dcache ? dcache->getInterface() : NULL;

    params.decodeToFetchDelay = decodeToFetchDelay;
    params.renameToFetchDelay = renameToFetchDelay;
    params.iewToFetchDelay = iewToFetchDelay;
    params.commitToFetchDelay = commitToFetchDelay;
    params.fetchWidth = fetchWidth;

    params.renameToDecodeDelay = renameToDecodeDelay;
    params.iewToDecodeDelay = iewToDecodeDelay;
    params.commitToDecodeDelay = commitToDecodeDelay;
    params.fetchToDecodeDelay = fetchToDecodeDelay;
    params.decodeWidth = decodeWidth;

    params.iewToRenameDelay = iewToRenameDelay;
    params.commitToRenameDelay = commitToRenameDelay;
    params.decodeToRenameDelay = decodeToRenameDelay;
    params.renameWidth = renameWidth;

    params.commitToIEWDelay = commitToIEWDelay;
    params.renameToIEWDelay = renameToIEWDelay;
    params.issueToExecuteDelay = issueToExecuteDelay;
    params.issueWidth = issueWidth;
    params.executeWidth = executeWidth;
    params.executeIntWidth = executeIntWidth;
    params.executeFloatWidth = executeFloatWidth;

    params.iewToCommitDelay = iewToCommitDelay;
    params.renameToROBDelay = renameToROBDelay;
    params.commitWidth = commitWidth;
    params.squashWidth = squashWidth;

    params.numPhysIntRegs = numPhysIntRegs;
    params.numPhysFloatRegs = numPhysFloatRegs;
    params.numIQEntries = numIQEntries;
    params.numROBEntries = numROBEntries;

    params.defReg = defReg;

    cpu = new AlphaFullCPU(params);

    return cpu;
}

REGISTER_SIM_OBJECT("AlphaFullCPU", AlphaFullCPU)

