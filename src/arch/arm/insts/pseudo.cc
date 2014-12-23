/*
 * Copyright (c) 2014 ARM Limited
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
 *
 * Authors: Andreas Sandberg
 */

#include "arch/arm/insts/pseudo.hh"
#include "cpu/exec_context.hh"

DecoderFaultInst::DecoderFaultInst(ExtMachInst _machInst)
    : ArmStaticInst("gem5decoderFault", _machInst, No_OpClass),
      faultId(static_cast<DecoderFault>(
                  static_cast<uint8_t>(_machInst.decoderFault)))
{
    // Don't call execute() if we're on a speculative path and the
    // fault is an internal panic fault.
    flags[IsNonSpeculative] = (faultId == DecoderFault::PANIC);
}

Fault
DecoderFaultInst::execute(ExecContext *xc, Trace::InstRecord *traceData) const
{
    const PCState pc_state(xc->pcState());
    const Addr pc(pc_state.instAddr());

    switch (faultId) {
      case DecoderFault::UNALIGNED:
        if (machInst.aarch64) {
            return std::make_shared<PCAlignmentFault>(pc);
        } else {
            // TODO: We should check if we the receiving end is in
            // aarch64 mode and raise a PCAlignment fault instead.
            return std::make_shared<PrefetchAbort>(
                pc, ArmFault::AlignmentFault);
        }

      case DecoderFault::PANIC:
        panic("Internal error in instruction decoder\n");

      case DecoderFault::OK:
        panic("Decoder fault instruction without decoder fault.\n");
    }

    panic("Unhandled fault type");
}

const char *
DecoderFaultInst::faultName() const
{
    switch (faultId) {
      case DecoderFault::OK:
        return "OK";

      case DecoderFault::UNALIGNED:
        return "UnalignedInstruction";

      case DecoderFault::PANIC:
        return "DecoderPanic";
    }

    panic("Unhandled fault type");
}

std::string
DecoderFaultInst::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    return csprintf("gem5fault %s", faultName());
}
