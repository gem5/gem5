/*
 * Copyright (c) 2024 Google LLC
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

#include "arch/riscv/insts/zcmt.hh"

#include <string>

#include "arch/riscv/pcstate.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/utility.hh"

namespace gem5
{

namespace RiscvISA
{

ZcmtSecondFetchInst::ZcmtSecondFetchInst(ExtMachInst machInst, Addr entry)
    : RiscvStaticInst("cm.jalt", machInst, IntAluOp), jvtEntry(entry)
{
    setRegIdxArrays(
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::srcRegIdxArr),
        reinterpret_cast<RegIdArrayPtr>(
            &std::remove_pointer_t<decltype(this)>::destRegIdxArr));
    flags[IsControl] = true;
    flags[IsDirectControl] = true;
    flags[IsInteger] = true;
    flags[IsUncondControl] = true;
}

Fault
ZcmtSecondFetchInst::execute(
    ExecContext *xc, trace::InstRecord *traceData) const
{
    PCState jvtPCState;
    set(jvtPCState, xc->pcState());
    jvtPCState.npc(rvSext(jvtEntry & ~0x1));
    jvtPCState.zcmtSecondFetch(false);
    jvtPCState.zcmtPc(0);
    xc->pcState(jvtPCState);
    return NoFault;
}

std::string
ZcmtSecondFetchInst::generateDisassembly(
    Addr pc, const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    ss << mnemonic << " jvt entry (" << std::hex << rvSext(jvtEntry & ~0x1)
       << ")";
    return ss.str();
}

std::unique_ptr<PCStateBase>
ZcmtSecondFetchInst::branchTarget(
    const PCStateBase &branch_pc) const
{
    auto &rpc = branch_pc.as<RiscvISA::PCState>();
    std::unique_ptr<PCState> npc(dynamic_cast<PCState*>(rpc.clone()));
    npc->zcmtSecondFetch(false);
    npc->zcmtPc(0);
    npc->set(rvSext(jvtEntry & ~0x1));
    return npc;
}

} // namespace RiscvISA
} // namespace gem5
