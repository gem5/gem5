/*
 * Copyright (c) 2020-2021 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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

#include "arch/arm/insts/tme64.hh"

#include <sstream>

#include "debug/ArmTme.hh"

namespace gem5
{

using namespace ArmISA;

namespace ArmISAInst
{

std::string
TmeImmOp64::generateDisassembly(Addr pc,
                                const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    ccprintf(ss, "#0x%x", imm);
    return ss.str();
}

std::string
TmeRegNone64::generateDisassembly(Addr pc,
                                  const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printIntReg(ss, dest);
    return ss.str();
}

std::string
MicroTmeBasic64::generateDisassembly(Addr pc,
                                     const loader::SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    return ss.str();
}

MicroTfence64::MicroTfence64(ExtMachInst machInst)
    : MicroTmeBasic64("utfence", machInst, MemReadOp)
{
    _numSrcRegs = 0;
    _numDestRegs = 0;
    flags[IsMicroop] = true;
    flags[IsReadBarrier] = true;
    flags[IsWriteBarrier] = true;
}

Fault
MicroTfence64::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    return NoFault;
}

Fault
MicroTfence64::initiateAcc(ExecContext *xc, trace::InstRecord *traceData) const
{
    panic("tfence should not have memory semantics");

    return NoFault;
}

Fault
MicroTfence64::completeAcc(PacketPtr pkt, ExecContext *xc,
                           trace::InstRecord *traceData) const
{
    panic("tfence should not have memory semantics");

    return NoFault;
}

Tstart64::Tstart64(ExtMachInst machInst, RegIndex _dest)
    : TmeRegNone64("tstart", machInst, MemReadOp, _dest)
{
    setRegIdxArrays(
        nullptr, reinterpret_cast<RegIdArrayPtr>(
                     &std::remove_pointer_t<decltype(this)>::destRegIdxArr));
    ;

    _numSrcRegs = 0;
    _numDestRegs = 0;
    setDestRegIdx(_numDestRegs++, intRegClass[dest]);
    _numTypedDestRegs[IntRegClass]++;
    flags[IsHtmStart] = true;
    flags[IsInteger] = true;
    flags[IsLoad] = true;
    flags[IsMicroop] = true;
    flags[IsNonSpeculative] = true;
}

Fault
Tstart64::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    panic("TME is not supported with atomic memory");

    return NoFault;
}

Ttest64::Ttest64(ExtMachInst machInst, RegIndex _dest)
    : TmeRegNone64("ttest", machInst, MemReadOp, _dest)
{
    setRegIdxArrays(
        nullptr, reinterpret_cast<RegIdArrayPtr>(
                     &std::remove_pointer_t<decltype(this)>::destRegIdxArr));
    ;

    _numSrcRegs = 0;
    _numDestRegs = 0;
    setDestRegIdx(_numDestRegs++, intRegClass[dest]);
    _numTypedDestRegs[IntRegClass]++;
    flags[IsInteger] = true;
    flags[IsMicroop] = true;
}

Tcancel64::Tcancel64(ExtMachInst machInst, uint64_t _imm)
    : TmeImmOp64("tcancel", machInst, MemReadOp, _imm)
{
    _numSrcRegs = 0;
    _numDestRegs = 0;
    flags[IsLoad] = true;
    flags[IsMicroop] = true;
    flags[IsNonSpeculative] = true;
    flags[IsHtmCancel] = true;
}

Fault
Tcancel64::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    panic("TME is not supported with atomic memory");

    return NoFault;
}

MacroTmeOp::MacroTmeOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass)
    : PredMacroOp(mnem, machInst, __opClass)
{
    _numSrcRegs = 0;
    _numDestRegs = 0;

    numMicroops = 0;
    microOps = nullptr;
}

MicroTcommit64::MicroTcommit64(ExtMachInst machInst)
    : MicroTmeBasic64("utcommit", machInst, MemReadOp)
{
    _numSrcRegs = 0;
    _numDestRegs = 0;
    flags[IsHtmStop] = true;
    flags[IsLoad] = true;
    flags[IsMicroop] = true;
    flags[IsNonSpeculative] = true;
}

Fault
MicroTcommit64::execute(ExecContext *xc, trace::InstRecord *traceData) const
{
    panic("TME is not supported with atomic memory");

    return NoFault;
}

Tcommit64::Tcommit64(ExtMachInst _machInst)
    : MacroTmeOp("tcommit", machInst, MemReadOp)
{
    numMicroops = 2;
    microOps = new StaticInstPtr[numMicroops];

    microOps[0] = new ArmISAInst::MicroTfence64(_machInst);
    microOps[0]->setDelayedCommit();
    microOps[0]->setFirstMicroop();

    microOps[1] = new ArmISAInst::MicroTcommit64(_machInst);
    microOps[1]->setDelayedCommit();
    microOps[1]->setLastMicroop();
}

} // namespace ArmISAInst
} // namespace gem5
