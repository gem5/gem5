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

#include <iomanip>

#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "enums/OpClass.hh"

using namespace std;
using namespace TheISA;

namespace Trace {

void
Trace::ExeTracerRecord::dump()
{
    ostream &outs = Trace::output();

    if (IsOn(ExecTicks))
        ccprintf(outs, "%7d: ", when);

    outs << thread->getCpuPtr()->name() << " ";

    if (IsOn(ExecSpeculative))
        outs << (misspeculating ? "-" : "+") << " ";

    if (IsOn(ExecThread))
        outs << "T" << thread->getThreadNum() << " : ";


    std::string sym_str;
    Addr sym_addr;
    if (debugSymbolTable
        && IsOn(ExecSymbol)
        && debugSymbolTable->findNearestSymbol(PC, sym_str, sym_addr)) {
        if (PC != sym_addr)
            sym_str += csprintf("+%d", PC - sym_addr);
        outs << "@" << sym_str << " : ";
    }
    else {
        outs << "0x" << hex << PC << " : ";
    }

    //
    //  Print decoded instruction
    //

    outs << setw(26) << left;
    outs << staticInst->disassemble(PC, debugSymbolTable);
    outs << " : ";

    if (IsOn(ExecOpClass)) {
        outs << Enums::OpClassStrings[staticInst->opClass()] << " : ";
    }

    if (IsOn(ExecResult) && data_status != DataInvalid) {
        ccprintf(outs, " D=%#018x", data.as_int);
    }

    if (IsOn(ExecEffAddr) && addr_valid)
        outs << " A=0x" << hex << addr;

    if (IsOn(ExecFetchSeq) && fetch_seq_valid)
        outs << "  FetchSeq=" << dec << fetch_seq;

    if (IsOn(ExecCPSeq) && cp_seq_valid)
        outs << "  CPSeq=" << dec << cp_seq;

    //
    //  End of line...
    //
    outs << endl;
}

/* namespace Trace */ }

////////////////////////////////////////////////////////////////////////
//
//  ExeTracer Simulation Object
//
Trace::ExeTracer *
ExeTracerParams::create()
{
    return new Trace::ExeTracer(this);
};
