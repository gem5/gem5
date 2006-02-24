/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#include "arch/alpha/alpha_memory.hh"
#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/osfpal.hh"
#include "base/kgdb.h"
#include "base/remote_gdb.hh"
#include "base/stats/events.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/exec_context.hh"
#include "cpu/fast/cpu.hh"
#include "kern/kernel_stats.hh"
#include "sim/debug.hh"
#include "sim/sim_events.hh"

#if FULL_SYSTEM

using namespace EV5;

////////////////////////////////////////////////////////////////////////
//
//
//
void
AlphaISA::swap_palshadow(RegFile *regs, bool use_shadow)
{
    if (regs->pal_shadow == use_shadow)
        panic("swap_palshadow: wrong PAL shadow state");

    regs->pal_shadow = use_shadow;

    for (int i = 0; i < NumIntRegs; i++) {
        if (reg_redir[i]) {
            IntReg temp = regs->intRegFile[i];
            regs->intRegFile[i] = regs->palregs[i];
            regs->palregs[i] = temp;
        }
    }
}

////////////////////////////////////////////////////////////////////////
//
//  Machine dependent functions
//
void
AlphaISA::initCPU(RegFile *regs, int cpuId)
{
    initIPRs(regs, cpuId);
    // CPU comes up with PAL regs enabled
    swap_palshadow(regs, true);

    regs->intRegFile[16] = cpuId;
    regs->intRegFile[0] = cpuId;

    regs->pc = regs->ipr[IPR_PAL_BASE] + fault_addr(ResetFault);
    regs->npc = regs->pc + sizeof(MachInst);
}

////////////////////////////////////////////////////////////////////////
//
// alpha exceptions - value equals trap address, update with MD_FAULT_TYPE
//
const Addr
AlphaISA::fault_addr(Fault fault)
{
        //Check for the system wide faults
        if(fault == NoFault) return 0x0000;
        else if(fault == MachineCheckFault) return 0x0401;
        else if(fault == AlignmentFault) return 0x0301;
        //Deal with the alpha specific faults
        return ((AlphaFault*)fault)->vect;
};

const int AlphaISA::reg_redir[AlphaISA::NumIntRegs] = {
    /*  0 */ 0, 0, 0, 0, 0, 0, 0, 0,
    /*  8 */ 1, 1, 1, 1, 1, 1, 1, 0,
    /* 16 */ 0, 0, 0, 0, 0, 0, 0, 0,
    /* 24 */ 0, 1, 0, 0, 0, 0, 0, 0 };

////////////////////////////////////////////////////////////////////////
//
//
//
void
AlphaISA::initIPRs(RegFile *regs, int cpuId)
{
    uint64_t *ipr = regs->ipr;

    bzero((char *)ipr, NumInternalProcRegs * sizeof(InternalProcReg));
    ipr[IPR_PAL_BASE] = PalBase;
    ipr[IPR_MCSR] = 0x6;
    ipr[IPR_PALtemp16] = cpuId;
}


template <class CPU>
void
AlphaISA::processInterrupts(CPU *cpu)
{
    //Check if there are any outstanding interrupts
    //Handle the interrupts
    int ipl = 0;
    int summary = 0;
    IntReg *ipr = cpu->getIprPtr();

    cpu->checkInterrupts = false;

    if (ipr[IPR_ASTRR])
        panic("asynchronous traps not implemented\n");

    if (ipr[IPR_SIRR]) {
        for (int i = INTLEVEL_SOFTWARE_MIN;
             i < INTLEVEL_SOFTWARE_MAX; i++) {
            if (ipr[IPR_SIRR] & (ULL(1) << i)) {
                // See table 4-19 of the 21164 hardware reference
                ipl = (i - INTLEVEL_SOFTWARE_MIN) + 1;
                summary |= (ULL(1) << i);
            }
        }
    }

    uint64_t interrupts = cpu->intr_status();

    if (interrupts) {
        for (int i = INTLEVEL_EXTERNAL_MIN;
             i < INTLEVEL_EXTERNAL_MAX; i++) {
            if (interrupts & (ULL(1) << i)) {
                // See table 4-19 of the 21164 hardware reference
                ipl = i;
                summary |= (ULL(1) << i);
            }
        }
    }

    if (ipl && ipl > ipr[IPR_IPLR]) {
        ipr[IPR_ISR] = summary;
        ipr[IPR_INTID] = ipl;
        cpu->trap(InterruptFault);
        DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                ipr[IPR_IPLR], ipl, summary);
    }

}

template <class CPU>
void
AlphaISA::zeroRegisters(CPU *cpu)
{
    // Insure ISA semantics
    // (no longer very clean due to the change in setIntReg() in the
    // cpu model.  Consider changing later.)
    cpu->xc->setIntReg(ZeroReg, 0);
    cpu->xc->setFloatRegDouble(ZeroReg, 0.0);
}

void
ExecContext::ev5_trap(Fault fault)
{
    DPRINTF(Fault, "Fault %s at PC: %#x\n", fault->name, regs.pc);
    cpu->recordEvent(csprintf("Fault %s", fault->name));

    assert(!misspeculating());
    kernelStats->fault(fault);

    if (fault == ArithmeticFault)
        panic("Arithmetic traps are unimplemented!");

    AlphaISA::InternalProcReg *ipr = regs.ipr;

    // exception restart address
    if (fault != InterruptFault || !inPalMode())
        ipr[AlphaISA::IPR_EXC_ADDR] = regs.pc;

    if (fault == PalFault || fault == ArithmeticFault /* ||
        fault == InterruptFault && !inPalMode() */) {
        // traps...  skip faulting instruction
        ipr[AlphaISA::IPR_EXC_ADDR] += 4;
    }

    if (!inPalMode())
        AlphaISA::swap_palshadow(&regs, true);

    regs.pc = ipr[AlphaISA::IPR_PAL_BASE] + AlphaISA::fault_addr(fault);
    regs.npc = regs.pc + sizeof(MachInst);
}


void
AlphaISA::intr_post(RegFile *regs, Fault fault, Addr pc)
{
    InternalProcReg *ipr = regs->ipr;
    bool use_pc = (fault == NoFault);

    if (fault == ArithmeticFault)
        panic("arithmetic faults NYI...");

    // compute exception restart address
    if (use_pc || fault == PalFault || fault == ArithmeticFault) {
        // traps...  skip faulting instruction
        ipr[IPR_EXC_ADDR] = regs->pc + 4;
    } else {
        // fault, post fault at excepting instruction
        ipr[IPR_EXC_ADDR] = regs->pc;
    }

    // jump to expection address (PAL PC bit set here as well...)
    if (!use_pc)
        regs->npc = ipr[IPR_PAL_BASE] + fault_addr(fault);
    else
        regs->npc = ipr[IPR_PAL_BASE] + pc;

    // that's it! (orders of magnitude less painful than x86)
}

Fault
ExecContext::hwrei()
{
    uint64_t *ipr = regs.ipr;

    if (!inPalMode())
        return UnimplementedOpcodeFault;

    setNextPC(ipr[AlphaISA::IPR_EXC_ADDR]);

    if (!misspeculating()) {
        kernelStats->hwrei();

        if ((ipr[AlphaISA::IPR_EXC_ADDR] & 1) == 0)
            AlphaISA::swap_palshadow(&regs, false);

        cpu->checkInterrupts = true;
    }

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

uint64_t
ExecContext::readIpr(int idx, Fault &fault)
{
    uint64_t *ipr = regs.ipr;
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
        retval |= cpu->curCycle()  & ULL(0x00000000ffffffff);
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
        fault = UnimplementedOpcodeFault;
        break;

      default:
        // invalid IPR
        fault = UnimplementedOpcodeFault;
        break;
    }

    return retval;
}

#ifdef DEBUG
// Cause the simulator to break when changing to the following IPL
int break_ipl = -1;
#endif

Fault
ExecContext::setIpr(int idx, uint64_t val)
{
    uint64_t *ipr = regs.ipr;
    uint64_t old;

    if (misspeculating())
        return NoFault;

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
        kernelStats->context(old, val);
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
        kernelStats->swpipl(ipr[idx]);
        break;

      case AlphaISA::IPR_DTB_CM:
        if (val & 0x18)
            kernelStats->mode(Kernel::user);
        else
            kernelStats->mode(Kernel::kernel);

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
        return UnimplementedOpcodeFault;

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
        return UnimplementedOpcodeFault;
    }

    // no error...
    return NoFault;
}

/**
 * Check for special simulator handling of specific PAL calls.
 * If return value is false, actual PAL call will be suppressed.
 */
bool
ExecContext::simPalCheck(int palFunc)
{
    kernelStats->callpal(palFunc);

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

//Forward instantiation for FastCPU object
template
void AlphaISA::processInterrupts(FastCPU *xc);

//Forward instantiation for FastCPU object
template
void AlphaISA::zeroRegisters(FastCPU *xc);

#endif // FULL_SYSTEM
