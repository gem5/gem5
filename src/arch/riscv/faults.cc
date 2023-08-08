/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * Copyright (c) 2018 TU Dresden
 * Copyright (c) 2020 Barkhausen Institut
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

#include "arch/riscv/faults.hh"

#include "arch/riscv/insts/static_inst.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/mmu.hh"
#include "arch/riscv/pmp.hh"
#include "arch/riscv/regs/misc.hh"
#include "arch/riscv/utility.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Faults.hh"
#include "sim/debug.hh"
#include "sim/full_system.hh"
#include "sim/workload.hh"

namespace gem5
{

namespace RiscvISA
{

void
RiscvFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Fault %s encountered at pc %s.", name(), tc->pcState());
}

void
RiscvFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    auto pc_state = tc->pcState().as<PCState>();

    DPRINTFS(Faults, tc->getCpuPtr(), "Fault (%s) at PC: %s\n",
             name(), pc_state);

    if (FullSystem) {
        PrivilegeMode pp = (PrivilegeMode)tc->readMiscReg(MISCREG_PRV);
        PrivilegeMode prv = PRV_M;
        STATUS status = tc->readMiscReg(MISCREG_STATUS);

        // According to riscv-privileged-v1.11, if a NMI occurs at the middle
        // of a M-mode trap handler, the state (epc/cause) will be overwritten
        // and is not necessary recoverable. There's nothing we can do here so
        // we'll just warn our user that the CPU state might be broken.
        warn_if(isNonMaskableInterrupt() && pp == PRV_M && status.mie == 0,
                "NMI overwriting M-mode trap handler state");

        // Set fault handler privilege mode
        if (isNonMaskableInterrupt()) {
            prv = PRV_M;
        } else if (isInterrupt()) {
            if (pp != PRV_M &&
                bits(tc->readMiscReg(MISCREG_MIDELEG), _code) != 0) {
                prv = PRV_S;
            }
            if (pp == PRV_U &&
                bits(tc->readMiscReg(MISCREG_SIDELEG), _code) != 0) {
                prv = PRV_U;
            }
        } else {
            if (pp != PRV_M &&
                bits(tc->readMiscReg(MISCREG_MEDELEG), _code) != 0) {
                prv = PRV_S;
            }
            if (pp == PRV_U &&
                bits(tc->readMiscReg(MISCREG_SEDELEG), _code) != 0) {
                prv = PRV_U;
            }
        }

        // Set fault registers and status
        MiscRegIndex cause, epc, tvec, tval;
        switch (prv) {
          case PRV_U:
            cause = MISCREG_UCAUSE;
            epc = MISCREG_UEPC;
            tvec = MISCREG_UTVEC;
            tval = MISCREG_UTVAL;

            status.upie = status.uie;
            status.uie = 0;
            break;
          case PRV_S:
            cause = MISCREG_SCAUSE;
            epc = MISCREG_SEPC;
            tvec = MISCREG_STVEC;
            tval = MISCREG_STVAL;

            status.spp = pp;
            status.spie = status.sie;
            status.sie = 0;
            break;
          case PRV_M:
            cause = MISCREG_MCAUSE;
            epc = MISCREG_MEPC;
            tvec = isNonMaskableInterrupt() ? MISCREG_NMIVEC : MISCREG_MTVEC;
            tval = MISCREG_MTVAL;

            status.mpp = pp;
            status.mpie = status.mie;
            status.mie = 0;
            break;
          default:
            panic("Unknown privilege mode %d.", prv);
            break;
        }

        // Set fault cause, privilege, and return PC
        uint64_t _cause = _code;
        if (isInterrupt()) {
           _cause |= CAUSE_INTERRUPT_MASKS[pc_state.rvType()];
        }
        tc->setMiscReg(cause, _cause);
        tc->setMiscReg(epc, tc->pcState().instAddr());
        tc->setMiscReg(tval, trap_value());
        tc->setMiscReg(MISCREG_PRV, prv);
        tc->setMiscReg(MISCREG_STATUS, status);
        // Temporarily mask NMI while we're in NMI handler. Otherweise, the
        // checkNonMaskableInterrupt will always return true and we'll be
        // stucked in an infinite loop.
        if (isNonMaskableInterrupt()) {
            tc->setMiscReg(MISCREG_NMIE, 0);
        }

        // Clear load reservation address
        auto isa = static_cast<RiscvISA::ISA*>(tc->getIsaPtr());
        isa->clearLoadReservation(tc->contextId());

        // Set PC to fault handler address
        Addr addr = mbits(tc->readMiscReg(tvec), 63, 2);
        if (isInterrupt() && bits(tc->readMiscReg(tvec), 1, 0) == 1)
            addr += 4 * _code;
        pc_state.set(addr);
        tc->pcState(pc_state);
    } else {
        invokeSE(tc, inst);
    }
}

void
Reset::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    tc->setMiscReg(MISCREG_PRV, PRV_M);
    STATUS status = tc->readMiscReg(MISCREG_STATUS);
    status.mie = 0;
    status.mprv = 0;
    tc->setMiscReg(MISCREG_STATUS, status);
    tc->setMiscReg(MISCREG_MCAUSE, 0);

    // Advance the PC to the implementation-defined reset vector
    auto workload = dynamic_cast<Workload *>(tc->getSystemPtr()->workload);
    std::unique_ptr<PCState> new_pc(dynamic_cast<PCState *>(
        tc->getIsaPtr()->newPCState(workload->getEntry())));
    panic_if(!new_pc, "Failed create new PCState from ISA pointer");
    tc->pcState(*new_pc);

    // Reset PMP Cfg
    auto* mmu = dynamic_cast<RiscvISA::MMU*>(tc->getMMUPtr());
    if (mmu == nullptr) {
        warn("MMU is not Riscv MMU instance, we can't reset PMP");
        return;
    }
    mmu->getPMP()->pmpReset();
}

void
UnknownInstFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    auto *rsi = static_cast<RiscvStaticInst *>(inst.get());
    panic("Unknown instruction 0x%08x at pc %s", rsi->machInst,
        tc->pcState());
}

void
IllegalInstFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (! tc->getSystemPtr()->trapToGdb(GDBSignal::ILL, tc->contextId()) ) {
        auto *rsi = static_cast<RiscvStaticInst *>(inst.get());
        panic("Illegal instruction 0x%08x at pc %s: %s", rsi->machInst,
            tc->pcState(), reason.c_str());
    }
}

void
UnimplementedFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Unimplemented instruction %s at pc %s", instName, tc->pcState());
}

void
IllegalFrmFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Illegal floating-point rounding mode 0x%x at pc %s.",
            frm, tc->pcState());
}

void
BreakpointFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (! tc->getSystemPtr()->trapToGdb(GDBSignal::TRAP, tc->contextId()) ) {
        schedRelBreak(0);
    }
}

void
SyscallFault::invokeSE(ThreadContext *tc, const StaticInstPtr &inst)
{
    /* Advance the PC to next instruction so - once (simulated) syscall
       is executed - execution continues. */
    auto pc_state = tc->pcState().as<PCState>();
    inst->advancePC(pc_state);
    tc->pcState(pc_state);

    tc->getSystemPtr()->workload->syscall(tc);
}

} // namespace RiscvISA
} // namespace gem5
