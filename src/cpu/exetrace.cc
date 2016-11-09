/*
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
 *
 * Authors: Steve Reinhardt
 *          Lisa Hsu
 *          Nathan Binkert
 *          Steve Raasch
 */

#include "cpu/exetrace.hh"

#include <iomanip>

#include "arch/isa_traits.hh"
#include "arch/utility.hh"
#include "base/loader/symtab.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/ExecAll.hh"
#include "enums/OpClass.hh"

using namespace std;
using namespace TheISA;

namespace Trace {

void
ExeTracerRecord::dumpTicks(ostream &outs)
{
    ccprintf(outs, "%7d: ", when);
}

void
Trace::ExeTracerRecord::traceInst(const StaticInstPtr &inst, bool ran)
{
    ostream &outs = Trace::output();

    if (!Debug::ExecUser || !Debug::ExecKernel) {
        bool in_user_mode = TheISA::inUserMode(thread);
        if (in_user_mode && !Debug::ExecUser) return;
        if (!in_user_mode && !Debug::ExecKernel) return;
    }

    if (Debug::ExecTicks)
        dumpTicks(outs);

    outs << thread->getCpuPtr()->name() << " ";

    if (Debug::ExecAsid)
        outs << "A" << dec << TheISA::getExecutingAsid(thread) << " ";

    if (Debug::ExecThread)
        outs << "T" << thread->threadId() << " : ";

    std::string sym_str;
    Addr sym_addr;
    Addr cur_pc = pc.instAddr();
    if (debugSymbolTable && Debug::ExecSymbol &&
            (!FullSystem || !inUserMode(thread)) &&
            debugSymbolTable->findNearestSymbol(cur_pc, sym_str, sym_addr)) {
        if (cur_pc != sym_addr)
            sym_str += csprintf("+%d",cur_pc - sym_addr);
        outs << "@" << sym_str;
    } else {
        outs << "0x" << hex << cur_pc;
    }

    if (inst->isMicroop()) {
        outs << "." << setw(2) << dec << pc.microPC();
    } else {
        outs << "   ";
    }

    outs << " : ";

    //
    //  Print decoded instruction
    //

    outs << setw(26) << left;
    outs << inst->disassemble(cur_pc, debugSymbolTable);

    if (ran) {
        outs << " : ";

        if (Debug::ExecOpClass) {
            outs << Enums::OpClassStrings[inst->opClass()] << " : ";
        }

        if (Debug::ExecResult && !predicate) {
            outs << "Predicated False";
        }

        if (Debug::ExecResult && data_status != DataInvalid) {
            ccprintf(outs, " D=%#018x", data.as_int);
        }

        if (Debug::ExecEffAddr && getMemValid())
            outs << " A=0x" << hex << addr;

        if (Debug::ExecFetchSeq && fetch_seq_valid)
            outs << "  FetchSeq=" << dec << fetch_seq;

        if (Debug::ExecCPSeq && cp_seq_valid)
            outs << "  CPSeq=" << dec << cp_seq;

        if (Debug::ExecFlags) {
            outs << "  flags=(";
            inst->printFlags(outs, "|");
            outs << ")";
        }
    }

    //
    //  End of line...
    //
    outs << endl;
}

void
Trace::ExeTracerRecord::dump()
{
    /*
     * The behavior this check tries to achieve is that if ExecMacro is on,
     * the macroop will be printed. If it's on and microops are also on, it's
     * printed before the microops start printing to give context. If the
     * microops aren't printed, then it's printed only when the final microop
     * finishes. Macroops then behave like regular instructions and don't
     * complete/print when they fault.
     */
    if (Debug::ExecMacro && staticInst->isMicroop() &&
        ((Debug::ExecMicro &&
            macroStaticInst && staticInst->isFirstMicroop()) ||
            (!Debug::ExecMicro &&
             macroStaticInst && staticInst->isLastMicroop()))) {
        traceInst(macroStaticInst, false);
    }
    if (Debug::ExecMicro || !staticInst->isMicroop()) {
        traceInst(staticInst, true);
    }
}

} // namespace Trace

////////////////////////////////////////////////////////////////////////
//
//  ExeTracer Simulation Object
//
Trace::ExeTracer *
ExeTracerParams::create()
{
    return new Trace::ExeTracer(this);
}
