/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <fstream>
#include <iomanip>

#include "dyn_inst.hh"
#include "spec_state.hh"
#include "issue.hh"
#include "exetrace.hh"
#include "exec_context.hh"
#include "symtab.hh"
#include "base_cpu.hh"
#include "static_inst.hh"

using namespace std;


////////////////////////////////////////////////////////////////////////
//
//  Methods for the InstRecord object
//


const SymbolTable *debugSymbolTable = NULL;

void
Trace::InstRecord::dump(ostream &outs)
{
    if (flags[PRINT_CYCLE])
        ccprintf(outs, "%7d: ", cycle);

    outs << cpu->name() << " ";

    if (flags[TRACE_MISSPEC])
        outs << (misspeculating ? "-" : "+") << " ";

    if (flags[PRINT_THREAD_NUM])
        outs << "T" << thread << " : ";

    outs << "0x" << hex << PC << " : ";

    //
    //  Print decoded instruction
    //

#if defined(__GNUC__) && (__GNUC__ < 3)
    // There's a bug in gcc 2.x library that prevents setw()
    // from working properly on strings
    string mc(staticInst->disassemble(PC, debugSymbolTable));
    while (mc.length() < 25)
        mc += " ";
    outs << mc;
#else
    outs << setw(25) << staticInst->disassemble(PC, debugSymbolTable);
#endif

    outs << " : ";

    if (flags[PRINT_OP_CLASS]) {
        outs << opClassStrings[staticInst->opClass()] << " : ";
    }

    if (flags[PRINT_RESULT_DATA] && data_status != DataInvalid) {
        outs << " D=";
#if 0
        if (data_status == DataDouble)
            ccprintf(outs, "%f", data.as_double);
        else
            ccprintf(outs, "%#018x", data.as_int);
#else
        ccprintf(outs, "%#018x", data.as_int);
#endif
    }

    if (flags[PRINT_EFF_ADDR] && addr_valid)
        outs << " A=0x" << hex << addr;

    if (flags[PRINT_INT_REGS] && regs_valid) {
        for (int i = 0; i < 32;)
            for (int j = i + 1; i <= j; i++)
                ccprintf(outs, "r%02d = %#018x%s", i, iregs->regs[i],
                         ((i == j) ? "\n" : "    "));
        outs << "\n";
    }

    if (flags[PRINT_FETCH_SEQ] && fetch_seq_valid)
        outs << "  FetchSeq=" << dec << fetch_seq;

    if (flags[PRINT_CP_SEQ] && cp_seq_valid)
        outs << "  CPSeq=" << dec << cp_seq;

    //
    //  End of line...
    //
    outs << endl;
    outs.flush();
}


vector<bool> Trace::InstRecord::flags(NUM_BITS);

////////////////////////////////////////////////////////////////////////
//
// Parameter space for per-cycle execution address tracing options.
// Derive from ParamContext so we can override checkParams() function.
//
class ExecutionTraceParamContext : public ParamContext
{
  public:
    ExecutionTraceParamContext(const string &_iniSection)
        : ParamContext(_iniSection)
        {
        }

    void checkParams();	// defined at bottom of file
};

ExecutionTraceParamContext exeTraceParams("exetrace");

Param<bool> exe_trace_spec(&exeTraceParams, "speculative",
                           "capture speculative instructions", false);

Param<bool> exe_trace_print_cycle(&exeTraceParams, "print_cycle",
                                  "print cycle number", true);
Param<bool> exe_trace_print_opclass(&exeTraceParams, "print_opclass",
                                  "print op class", true);
Param<bool> exe_trace_print_thread(&exeTraceParams, "print_thread",
                                  "print thread number", true);
Param<bool> exe_trace_print_effaddr(&exeTraceParams, "print_effaddr",
                                  "print effective address", true);
Param<bool> exe_trace_print_data(&exeTraceParams, "print_data",
                                  "print result data", true);
Param<bool> exe_trace_print_iregs(&exeTraceParams, "print_iregs",
                                  "print all integer regs", false);
Param<bool> exe_trace_print_fetchseq(&exeTraceParams, "print_fetchseq",
                                  "print fetch sequence number", false);
Param<bool> exe_trace_print_cp_seq(&exeTraceParams, "print_cpseq",
                                  "print correct-path sequence number", false);

//
// Helper function for ExecutionTraceParamContext::checkParams() just
// to get us into the InstRecord namespace
//
void
Trace::InstRecord::setParams()
{
    flags[TRACE_MISSPEC]     = exe_trace_spec;

    flags[PRINT_CYCLE]       = exe_trace_print_cycle;
    flags[PRINT_OP_CLASS]    = exe_trace_print_opclass;
    flags[PRINT_THREAD_NUM]  = exe_trace_print_thread;
    flags[PRINT_RESULT_DATA] = exe_trace_print_effaddr;
    flags[PRINT_EFF_ADDR]    = exe_trace_print_data;
    flags[PRINT_INT_REGS]    = exe_trace_print_iregs;
    flags[PRINT_FETCH_SEQ]   = exe_trace_print_fetchseq;
    flags[PRINT_CP_SEQ]      = exe_trace_print_cp_seq;
}

void
ExecutionTraceParamContext::checkParams()
{
    Trace::InstRecord::setParams();
}

