/*
 * Copyright (c) 2017, 2019 ARM Limited
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

#include "arch/utility.hh"
#include "base/loader/symtab.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/ExecAll.hh"
#include "debug/FmtTicksOff.hh"
#include "enums/OpClass.hh"

using namespace std;
using namespace TheISA;

namespace Trace {

void
Trace::ExeTracerRecord::traceInst(const StaticInstPtr &inst, bool ran)
{
    std::stringstream outs;

    if (!Debug::ExecUser || !Debug::ExecKernel) {
        bool in_user_mode = TheISA::inUserMode(thread);
        if (in_user_mode && !Debug::ExecUser) return;
        if (!in_user_mode && !Debug::ExecKernel) return;
    }

    if (Debug::ExecAsid)
        outs << "A" << dec << TheISA::getExecutingAsid(thread) << " ";

    if (Debug::ExecThread)
        outs << "T" << thread->threadId() << " : ";

    Addr cur_pc = pc.instAddr();
    Loader::SymbolTable::const_iterator it;
    if (Debug::ExecSymbol && (!FullSystem || !inUserMode(thread)) &&
            (it = Loader::debugSymbolTable.findNearest(cur_pc)) !=
                Loader::debugSymbolTable.end()) {
        Addr delta = cur_pc - it->address;
        if (delta)
            ccprintf(outs, "@%s+%d", it->name, delta);
        else
            ccprintf(outs, "@%s", it->name);
    } else {
        ccprintf(outs, "%#x", cur_pc);
    }

    if (inst->isMicroop()) {
        ccprintf(outs, ".%2d", pc.microPC());
    } else {
        ccprintf(outs, "   ");
    }

    ccprintf(outs, " : ");

    //
    //  Print decoded instruction
    //

    outs << setw(26) << left;
    outs << inst->disassemble(cur_pc, &Loader::debugSymbolTable);

    if (ran) {
        outs << " : ";

        if (Debug::ExecOpClass) {
            outs << Enums::OpClassStrings[inst->opClass()] << " : ";
        }

        if (Debug::ExecResult && !predicate) {
            outs << "Predicated False";
        }

        if (Debug::ExecResult && data_status != DataInvalid) {
            switch (data_status) {
              case DataVec:
                {
                    ccprintf(outs, " D=0x[");
                    auto dv = data.as_vec->as<uint32_t>();
                    for (int i = TheISA::VecRegSizeBytes / 4 - 1; i >= 0;
                         i--) {
                        ccprintf(outs, "%08x", dv[i]);
                        if (i != 0) {
                            ccprintf(outs, "_");
                        }
                    }
                    ccprintf(outs, "]");
                }
                break;
              case DataVecPred:
                {
                    ccprintf(outs, " D=0b[");
                    auto pv = data.as_pred->as<uint8_t>();
                    for (int i = TheISA::VecPredRegSizeBits - 1; i >= 0; i--) {
                        ccprintf(outs, pv[i] ? "1" : "0");
                        if (i != 0 && i % 4 == 0) {
                            ccprintf(outs, "_");
                        }
                    }
                    ccprintf(outs, "]");
                }
                break;
              default:
                ccprintf(outs, " D=%#018x", data.as_int);
                break;
            }
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

    Trace::getDebugLogger()->dprintf_flag(
        when, thread->getCpuPtr()->name(), "ExecEnable", "%s",
        outs.str().c_str());
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
