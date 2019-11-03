/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 *          Kevin Lim
 */

#include "arch/alpha/faults.hh"

#include "arch/alpha/ev5.hh"
#include "arch/alpha/tlb.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/page_table.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

namespace AlphaISA {

FaultName MachineCheckFault::_name = "mchk";
FaultVect MachineCheckFault::_vect = 0x0401;
FaultStat MachineCheckFault::_count;

FaultName AlignmentFault::_name = "unalign";
FaultVect AlignmentFault::_vect = 0x0301;
FaultStat AlignmentFault::_count;

FaultName ResetFault::_name = "reset";
FaultVect ResetFault::_vect = 0x0001;
FaultStat ResetFault::_count;

FaultName ArithmeticFault::_name = "arith";
FaultVect ArithmeticFault::_vect = 0x0501;
FaultStat ArithmeticFault::_count;

FaultName InterruptFault::_name = "interrupt";
FaultVect InterruptFault::_vect = 0x0101;
FaultStat InterruptFault::_count;

FaultName NDtbMissFault::_name = "dtb_miss_single";
FaultVect NDtbMissFault::_vect = 0x0201;
FaultStat NDtbMissFault::_count;

FaultName PDtbMissFault::_name = "dtb_miss_double";
FaultVect PDtbMissFault::_vect = 0x0281;
FaultStat PDtbMissFault::_count;

FaultName DtbPageFault::_name = "dtb_page_fault";
FaultVect DtbPageFault::_vect = 0x0381;
FaultStat DtbPageFault::_count;

FaultName DtbAcvFault::_name = "dtb_acv_fault";
FaultVect DtbAcvFault::_vect = 0x0381;
FaultStat DtbAcvFault::_count;

FaultName DtbAlignmentFault::_name = "unalign";
FaultVect DtbAlignmentFault::_vect = 0x0301;
FaultStat DtbAlignmentFault::_count;

FaultName ItbPageFault::_name = "itbmiss";
FaultVect ItbPageFault::_vect = 0x0181;
FaultStat ItbPageFault::_count;

FaultName ItbAcvFault::_name = "iaccvio";
FaultVect ItbAcvFault::_vect = 0x0081;
FaultStat ItbAcvFault::_count;

FaultName UnimplementedOpcodeFault::_name = "opdec";
FaultVect UnimplementedOpcodeFault::_vect = 0x0481;
FaultStat UnimplementedOpcodeFault::_count;

FaultName FloatEnableFault::_name = "fen";
FaultVect FloatEnableFault::_vect = 0x0581;
FaultStat FloatEnableFault::_count;

/* We use the same fault vector, as for the guest system these should be the
 * same, but for host purposes, having differentiation is helpful for
 * debug/monitorization purposes. */
FaultName VectorEnableFault::_name = "ven";
FaultVect VectorEnableFault::_vect = 0x0581;
FaultStat VectorEnableFault::_count;

FaultName PalFault::_name = "pal";
FaultVect PalFault::_vect = 0x2001;
FaultStat PalFault::_count;

FaultName IntegerOverflowFault::_name = "intover";
FaultVect IntegerOverflowFault::_vect = 0x0501;
FaultStat IntegerOverflowFault::_count;

void
AlphaFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    FaultBase::invoke(tc);
    if (!FullSystem)
        return;
    countStat()++;

    PCState pc = tc->pcState();

    // exception restart address
    if (setRestartAddress() || !(pc.pc() & 0x3))
        tc->setMiscRegNoEffect(IPR_EXC_ADDR, pc.pc());

    if (skipFaultingInstruction()) {
        // traps...  skip faulting instruction.
        tc->setMiscRegNoEffect(IPR_EXC_ADDR,
                   tc->readMiscRegNoEffect(IPR_EXC_ADDR) + 4);
    }

    pc.set(tc->readMiscRegNoEffect(IPR_PAL_BASE) + vect());
    tc->pcState(pc);
}

void
ArithmeticFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    FaultBase::invoke(tc);
    if (!FullSystem)
        return;
    panic("Arithmetic traps are unimplemented!");
}

void
DtbFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        // Set fault address and flags.  Even though we're modeling an
        // EV5, we use the EV6 technique of not latching fault registers
        // on VPTE loads (instead of locking the registers until IPR_VA is
        // read, like the EV5).  The EV6 approach is cleaner and seems to
        // work with EV5 PAL code, but not the other way around.
        if (reqFlags.noneSet(AlphaRequestFlags::VPTE | Request::PREFETCH)) {
            // set VA register with faulting address
            tc->setMiscRegNoEffect(IPR_VA, vaddr);

            // set MM_STAT register flags
            MachInst machInst = inst->machInst;
            tc->setMiscRegNoEffect(IPR_MM_STAT,
                (((Opcode(machInst) & 0x3f) << 11) |
                 ((Ra(machInst) & 0x1f) << 6) |
                 (flags & 0x3f)));

            // set VA_FORM register with faulting formatted address
            tc->setMiscRegNoEffect(IPR_VA_FORM,
                tc->readMiscRegNoEffect(IPR_MVPTBR) | (vaddr.vpn() << 3));
        }
    }

    AlphaFault::invoke(tc);
}

void
ItbFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        tc->setMiscRegNoEffect(IPR_ITB_TAG, pc);
        tc->setMiscRegNoEffect(IPR_IFAULT_VA_FORM,
            tc->readMiscRegNoEffect(IPR_IVPTBR) | (VAddr(pc).vpn() << 3));
    }

    AlphaFault::invoke(tc);
}

void
ItbPageFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        ItbFault::invoke(tc);
        return;
    }

    Process *p = tc->getProcessPtr();
    const EmulationPageTable::Entry *pte = p->pTable->lookup(pc);
    panic_if(!pte, "Tried to execute unmapped address %#x.\n", pc);

    VAddr vaddr(pc);
    TlbEntry entry(p->pTable->pid(), vaddr.page(), pte->paddr,
                   pte->flags & EmulationPageTable::Uncacheable,
                   pte->flags & EmulationPageTable::ReadOnly);
    dynamic_cast<TLB *>(tc->getITBPtr())->insert(vaddr.page(), entry);
}

void
NDtbMissFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        DtbFault::invoke(tc, inst);
        return;
    }

    Process *p = tc->getProcessPtr();
    const EmulationPageTable::Entry *pte = p->pTable->lookup(vaddr);
    if (!pte && p->fixupStackFault(vaddr))
        pte = p->pTable->lookup(vaddr);
    panic_if(!pte, "Tried to access unmapped address %#x.\n", (Addr)vaddr);
    TlbEntry entry(p->pTable->pid(), vaddr.page(), pte->paddr,
                   pte->flags & EmulationPageTable::Uncacheable,
                   pte->flags & EmulationPageTable::ReadOnly);
    dynamic_cast<TLB *>(tc->getDTBPtr())->insert(vaddr.page(), entry);
}

} // namespace AlphaISA

