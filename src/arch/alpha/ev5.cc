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
 *
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

#include "arch/alpha/faults.hh"
#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/kernel_stats.hh"
#include "arch/alpha/osfpal.hh"
#include "arch/alpha/tlb.hh"
#include "base/cp_annotate.hh"
#include "base/debug.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "sim/sim_exit.hh"

namespace AlphaISA {

////////////////////////////////////////////////////////////////////////
//
//  Machine dependent functions
//
void
initCPU(ThreadContext *tc, int cpuId)
{
    initIPRs(tc, cpuId);

    tc->setIntReg(16, cpuId);
    tc->setIntReg(0, cpuId);

    AlphaFault *reset = new ResetFault;

    tc->pcState(tc->readMiscRegNoEffect(IPR_PAL_BASE) + reset->vect());

    delete reset;
}

template <class CPU>
void
zeroRegisters(CPU *cpu)
{
    // Insure ISA semantics
    // (no longer very clean due to the change in setIntReg() in the
    // cpu model.  Consider changing later.)
    cpu->thread->setIntReg(ZeroReg, 0);
    cpu->thread->setFloatReg(ZeroReg, 0.0);
}

////////////////////////////////////////////////////////////////////////
//
//
//
void
initIPRs(ThreadContext *tc, int cpuId)
{
    for (int i = 0; i < NumInternalProcRegs; ++i) {
        tc->setMiscRegNoEffect(i, 0);
    }

    tc->setMiscRegNoEffect(IPR_PAL_BASE, PalBase);
    tc->setMiscRegNoEffect(IPR_MCSR, 0x6);
    tc->setMiscRegNoEffect(IPR_PALtemp16, cpuId);
}

MiscReg
ISA::readIpr(int idx, ThreadContext *tc)
{
    uint64_t retval = 0;        // return value, default 0

    switch (idx) {
      case IPR_PALtemp0:
      case IPR_PALtemp1:
      case IPR_PALtemp2:
      case IPR_PALtemp3:
      case IPR_PALtemp4:
      case IPR_PALtemp5:
      case IPR_PALtemp6:
      case IPR_PALtemp7:
      case IPR_PALtemp8:
      case IPR_PALtemp9:
      case IPR_PALtemp10:
      case IPR_PALtemp11:
      case IPR_PALtemp12:
      case IPR_PALtemp13:
      case IPR_PALtemp14:
      case IPR_PALtemp15:
      case IPR_PALtemp16:
      case IPR_PALtemp17:
      case IPR_PALtemp18:
      case IPR_PALtemp19:
      case IPR_PALtemp20:
      case IPR_PALtemp21:
      case IPR_PALtemp22:
      case IPR_PALtemp23:
      case IPR_PAL_BASE:

      case IPR_IVPTBR:
      case IPR_DC_MODE:
      case IPR_MAF_MODE:
      case IPR_ISR:
      case IPR_EXC_ADDR:
      case IPR_IC_PERR_STAT:
      case IPR_DC_PERR_STAT:
      case IPR_MCSR:
      case IPR_ASTRR:
      case IPR_ASTER:
      case IPR_SIRR:
      case IPR_ICSR:
      case IPR_ICM:
      case IPR_DTB_CM:
      case IPR_IPLR:
      case IPR_INTID:
      case IPR_PMCTR:
        // no side-effect
        retval = ipr[idx];
        break;

      case IPR_CC:
        retval |= ipr[idx] & ULL(0xffffffff00000000);
        retval |= tc->getCpuPtr()->curCycle()  & ULL(0x00000000ffffffff);
        break;

      case IPR_VA:
        retval = ipr[idx];
        break;

      case IPR_VA_FORM:
      case IPR_MM_STAT:
      case IPR_IFAULT_VA_FORM:
      case IPR_EXC_MASK:
      case IPR_EXC_SUM:
        retval = ipr[idx];
        break;

      case IPR_DTB_PTE:
        {
            TlbEntry &entry = tc->getDTBPtr()->index(1);

            retval |= ((uint64_t)entry.ppn & ULL(0x7ffffff)) << 32;
            retval |= ((uint64_t)entry.xre & ULL(0xf)) << 8;
            retval |= ((uint64_t)entry.xwe & ULL(0xf)) << 12;
            retval |= ((uint64_t)entry.fonr & ULL(0x1)) << 1;
            retval |= ((uint64_t)entry.fonw & ULL(0x1))<< 2;
            retval |= ((uint64_t)entry.asma & ULL(0x1)) << 4;
            retval |= ((uint64_t)entry.asn & ULL(0x7f)) << 57;
        }
        break;

        // write only registers
      case IPR_HWINT_CLR:
      case IPR_SL_XMIT:
      case IPR_DC_FLUSH:
      case IPR_IC_FLUSH:
      case IPR_ALT_MODE:
      case IPR_DTB_IA:
      case IPR_DTB_IAP:
      case IPR_ITB_IA:
      case IPR_ITB_IAP:
        panic("Tried to read write only register %d\n", idx);
        break;

      default:
        // invalid IPR
        panic("Tried to read from invalid ipr %d\n", idx);
        break;
    }

    return retval;
}

// Cause the simulator to break when changing to the following IPL
int break_ipl = -1;

void
ISA::setIpr(int idx, uint64_t val, ThreadContext *tc)
{
    switch (idx) {
      case IPR_PALtemp0:
      case IPR_PALtemp1:
      case IPR_PALtemp2:
      case IPR_PALtemp3:
      case IPR_PALtemp4:
      case IPR_PALtemp5:
      case IPR_PALtemp6:
      case IPR_PALtemp7:
      case IPR_PALtemp8:
      case IPR_PALtemp9:
      case IPR_PALtemp10:
      case IPR_PALtemp11:
      case IPR_PALtemp12:
      case IPR_PALtemp13:
      case IPR_PALtemp14:
      case IPR_PALtemp15:
      case IPR_PALtemp16:
      case IPR_PALtemp17:
      case IPR_PALtemp18:
      case IPR_PALtemp19:
      case IPR_PALtemp20:
      case IPR_PALtemp21:
      case IPR_PALtemp22:
      case IPR_PAL_BASE:
      case IPR_IC_PERR_STAT:
      case IPR_DC_PERR_STAT:
      case IPR_PMCTR:
        // write entire quad w/ no side-effect
        ipr[idx] = val;
        break;

      case IPR_CC_CTL:
        // This IPR resets the cycle counter.  We assume this only
        // happens once... let's verify that.
        assert(ipr[idx] == 0);
        ipr[idx] = 1;
        break;

      case IPR_CC:
        // This IPR only writes the upper 64 bits.  It's ok to write
        // all 64 here since we mask out the lower 32 in rpcc (see
        // isa_desc).
        ipr[idx] = val;
        break;

      case IPR_PALtemp23:
        // write entire quad w/ no side-effect
        if (tc->getKernelStats())
            tc->getKernelStats()->context(ipr[idx], val, tc);
        ipr[idx] = val;
        break;

      case IPR_DTB_PTE:
        // write entire quad w/ no side-effect, tag is forthcoming
        ipr[idx] = val;
        break;

      case IPR_EXC_ADDR:
        // second least significant bit in PC is always zero
        ipr[idx] = val & ~2;
        break;

      case IPR_ASTRR:
      case IPR_ASTER:
        // only write least significant four bits - privilege mask
        ipr[idx] = val & 0xf;
        break;

      case IPR_IPLR:
#ifdef DEBUG
        if (break_ipl != -1 && break_ipl == (int)(val & 0x1f))
            Debug::breakpoint();
#endif

        // only write least significant five bits - interrupt level
        ipr[idx] = val & 0x1f;
        if (tc->getKernelStats())
            tc->getKernelStats()->swpipl(ipr[idx]);
        break;

      case IPR_DTB_CM:
        if (val & 0x18) {
            if (tc->getKernelStats())
                tc->getKernelStats()->mode(Kernel::user, tc);
        } else {
            if (tc->getKernelStats())
                tc->getKernelStats()->mode(Kernel::kernel, tc);
        }

      case IPR_ICM:
        // only write two mode bits - processor mode
        ipr[idx] = val & 0x18;
        break;

      case IPR_ALT_MODE:
        // only write two mode bits - processor mode
        ipr[idx] = val & 0x18;
        break;

      case IPR_MCSR:
        // more here after optimization...
        ipr[idx] = val;
        break;

      case IPR_SIRR:
        // only write software interrupt mask
        ipr[idx] = val & 0x7fff0;
        break;

      case IPR_ICSR:
        ipr[idx] = val & ULL(0xffffff0300);
        break;

      case IPR_IVPTBR:
      case IPR_MVPTBR:
        ipr[idx] = val & ULL(0xffffffffc0000000);
        break;

      case IPR_DC_TEST_CTL:
        ipr[idx] = val & 0x1ffb;
        break;

      case IPR_DC_MODE:
      case IPR_MAF_MODE:
        ipr[idx] = val & 0x3f;
        break;

      case IPR_ITB_ASN:
        ipr[idx] = val & 0x7f0;
        break;

      case IPR_DTB_ASN:
        ipr[idx] = val & ULL(0xfe00000000000000);
        break;

      case IPR_EXC_SUM:
      case IPR_EXC_MASK:
        // any write to this register clears it
        ipr[idx] = 0;
        break;

      case IPR_INTID:
      case IPR_SL_RCV:
      case IPR_MM_STAT:
      case IPR_ITB_PTE_TEMP:
      case IPR_DTB_PTE_TEMP:
        // read-only registers
        panic("Tried to write read only ipr %d\n", idx);

      case IPR_HWINT_CLR:
      case IPR_SL_XMIT:
      case IPR_DC_FLUSH:
      case IPR_IC_FLUSH:
        // the following are write only
        ipr[idx] = val;
        break;

      case IPR_DTB_IA:
        // really a control write
        ipr[idx] = 0;

        tc->getDTBPtr()->flushAll();
        break;

      case IPR_DTB_IAP:
        // really a control write
        ipr[idx] = 0;

        tc->getDTBPtr()->flushProcesses();
        break;

      case IPR_DTB_IS:
        // really a control write
        ipr[idx] = val;

        tc->getDTBPtr()->flushAddr(val, DTB_ASN_ASN(ipr[IPR_DTB_ASN]));
        break;

      case IPR_DTB_TAG: {
          struct TlbEntry entry;

          // FIXME: granularity hints NYI...
          if (DTB_PTE_GH(ipr[IPR_DTB_PTE]) != 0)
              panic("PTE GH field != 0");

          // write entire quad
          ipr[idx] = val;

          // construct PTE for new entry
          entry.ppn = DTB_PTE_PPN(ipr[IPR_DTB_PTE]);
          entry.xre = DTB_PTE_XRE(ipr[IPR_DTB_PTE]);
          entry.xwe = DTB_PTE_XWE(ipr[IPR_DTB_PTE]);
          entry.fonr = DTB_PTE_FONR(ipr[IPR_DTB_PTE]);
          entry.fonw = DTB_PTE_FONW(ipr[IPR_DTB_PTE]);
          entry.asma = DTB_PTE_ASMA(ipr[IPR_DTB_PTE]);
          entry.asn = DTB_ASN_ASN(ipr[IPR_DTB_ASN]);

          // insert new TAG/PTE value into data TLB
          tc->getDTBPtr()->insert(val, entry);
      }
        break;

      case IPR_ITB_PTE: {
          struct TlbEntry entry;

          // FIXME: granularity hints NYI...
          if (ITB_PTE_GH(val) != 0)
              panic("PTE GH field != 0");

          // write entire quad
          ipr[idx] = val;

          // construct PTE for new entry
          entry.ppn = ITB_PTE_PPN(val);
          entry.xre = ITB_PTE_XRE(val);
          entry.xwe = 0;
          entry.fonr = ITB_PTE_FONR(val);
          entry.fonw = ITB_PTE_FONW(val);
          entry.asma = ITB_PTE_ASMA(val);
          entry.asn = ITB_ASN_ASN(ipr[IPR_ITB_ASN]);

          // insert new TAG/PTE value into data TLB
          tc->getITBPtr()->insert(ipr[IPR_ITB_TAG], entry);
      }
        break;

      case IPR_ITB_IA:
        // really a control write
        ipr[idx] = 0;

        tc->getITBPtr()->flushAll();
        break;

      case IPR_ITB_IAP:
        // really a control write
        ipr[idx] = 0;

        tc->getITBPtr()->flushProcesses();
        break;

      case IPR_ITB_IS:
        // really a control write
        ipr[idx] = val;

        tc->getITBPtr()->flushAddr(val, ITB_ASN_ASN(ipr[IPR_ITB_ASN]));
        break;

      default:
        // invalid IPR
        panic("Tried to write to invalid ipr %d\n", idx);
    }

    // no error...
}

void
copyIprs(ThreadContext *src, ThreadContext *dest)
{
    for (int i = 0; i < NumInternalProcRegs; ++i)
        dest->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));
}

} // namespace AlphaISA

using namespace AlphaISA;

Fault
SimpleThread::hwrei()
{
    PCState pc = pcState();
    if (!(pc.pc() & 0x3))
        return std::make_shared<UnimplementedOpcodeFault>();

    pc.npc(readMiscRegNoEffect(IPR_EXC_ADDR));
    pcState(pc);

    CPA::cpa()->swAutoBegin(tc, pc.npc());

    if (kernelStats)
        kernelStats->hwrei();

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

/**
 * Check for special simulator handling of specific PAL calls.
 * If return value is false, actual PAL call will be suppressed.
 */
bool
SimpleThread::simPalCheck(int palFunc)
{
    if (kernelStats)
        kernelStats->callpal(palFunc, tc);

    switch (palFunc) {
      case PAL::halt:
        halt();
        if (--System::numSystemsRunning == 0)
            exitSimLoop("all cpus halted");
        break;

      case PAL::bpt:
      case PAL::bugchk:
        if (system->breakpoint())
            return false;
        break;
    }

    return true;
}
