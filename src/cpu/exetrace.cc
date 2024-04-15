/*
 * Copyright (c) 2017, 2019, 2023 Arm Limited
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
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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

#include "cpu/exetrace.hh"

#include <iomanip>
#include <sstream>

#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/ExecAll.hh"
#include "debug/FmtTicksOff.hh"
#include "enums/OpClass.hh"

namespace gem5
{

namespace trace
{

void
ExeTracerRecord::traceInst(const StaticInstPtr &inst, bool ran)
{
    std::stringstream outs;

    const bool in_user_mode = thread->getIsaPtr()->inUserMode();
    if (in_user_mode && !debug::ExecUser)
        return;
    if (!in_user_mode && !debug::ExecKernel)
        return;

    if (debug::ExecAsid) {
        outs << "A" << std::dec << thread->getIsaPtr()->getExecutingAsid()
             << " ";
    }

    if (debug::ExecThread)
        outs << "T" << thread->threadId() << " : ";

    Addr cur_pc = pc->instAddr();
    loader::SymbolTable::const_iterator it;
    ccprintf(outs, "%#x", cur_pc);
    if (debug::ExecSymbol && (!FullSystem || !in_user_mode) &&
        (it = loader::debugSymbolTable.findNearest(cur_pc)) !=
            loader::debugSymbolTable.end()) {
        Addr delta = cur_pc - it->address();
        if (delta)
            ccprintf(outs, " @%s+%d", it->name(), delta);
        else
            ccprintf(outs, " @%s", it->name());
    }

    if (inst->isMicroop()) {
        ccprintf(outs, ".%2d", pc->microPC());
    } else {
        ccprintf(outs, "   ");
    }

    ccprintf(outs, " : ");

    //
    //  Print decoded instruction
    //

    outs << std::setw(26) << std::left;
    outs << tracer.disassemble(inst, *pc, &loader::debugSymbolTable);

    if (ran) {
        outs << " : ";

        if (debug::ExecOpClass) {
            outs << enums::OpClassStrings[inst->opClass()] << " : ";
        }

        if (debug::ExecResult && !predicate) {
            outs << "Predicated False";
        }

        if (debug::ExecResult && dataStatus != DataInvalid) {
            if (dataStatus == DataReg)
                ccprintf(outs, " D=%s", data.asReg.asString());
            else
                ccprintf(outs, " D=%#018x", data.asInt);
        }

        if (debug::ExecEffAddr && getMemValid())
            outs << " A=0x" << std::hex << addr;

        if (debug::ExecFetchSeq && fetch_seq_valid)
            outs << "  FetchSeq=" << std::dec << fetch_seq;

        if (debug::ExecCPSeq && cp_seq_valid)
            outs << "  CPSeq=" << std::dec << cp_seq;

        if (debug::ExecFlags) {
            outs << "  flags=(";
            inst->printFlags(outs, "|");
            outs << ")";
        }
    }

    //
    //  End of line...
    //
    outs << std::endl;

    trace::getDebugLogger()->dprintf_flag(when, thread->getCpuPtr()->name(),
                                          "ExecEnable", "%s",
                                          outs.str().c_str());
}

void
ExeTracerRecord::dump()
{
    /*
     * The behavior this check tries to achieve is that if ExecMacro is on,
     * the macroop will be printed. If it's on and microops are also on, it's
     * printed before the microops start printing to give context. If the
     * microops aren't printed, then it's printed only when the final microop
     * finishes. Macroops then behave like regular instructions and don't
     * complete/print when they fault.
     */
    if (debug::ExecMacro && staticInst->isMicroop() &&
        ((debug::ExecMicro && macroStaticInst &&
          staticInst->isFirstMicroop()) ||
         (!debug::ExecMicro && macroStaticInst &&
          staticInst->isLastMicroop()))) {
        traceInst(macroStaticInst, false);
    }
    if (debug::ExecMicro || !staticInst->isMicroop()) {
        traceInst(staticInst, true);
    }
}

} // namespace trace
} // namespace gem5
