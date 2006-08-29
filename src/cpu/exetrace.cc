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

#include <fstream>
#include <iomanip>

#include "arch/regfile.hh"
#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/static_inst.hh"
#include "sim/param.hh"
#include "sim/system.hh"

using namespace std;
using namespace TheISA;

////////////////////////////////////////////////////////////////////////
//
//  Methods for the InstRecord object
//


void
Trace::InstRecord::dump(ostream &outs)
{
    static uint64_t regs[32] = {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0};
    static uint64_t ccr = 0;
    static uint64_t y = 0;
    static uint64_t floats[32];
    uint64_t newVal;
    static const char * prefixes[4] = {"G", "O", "L", "I"};
    if (flags[PRINT_REG_DELTA])
    {
        ThreadContext * context = cpu->threadContexts[0];
        char buf[256];
        sprintf(buf, "PC = 0x%016llx", context->readNextPC());
        outs << buf;
        sprintf(buf, " NPC = 0x%016llx", context->readNextNPC());
        outs << buf;
        newVal = context->readMiscReg(SparcISA::MISCREG_CCR);
        if(newVal != ccr)
        {
            sprintf(buf, " CCR = 0x%016llx", newVal);
            outs << buf;
            ccr = newVal;
        }
        newVal = context->readMiscReg(SparcISA::MISCREG_Y);
        if(newVal != y)
        {
            sprintf(buf, " Y = 0x%016llx", newVal);
            outs << buf;
            y = newVal;
        }
        for(int y = 0; y < 4; y++)
        {
            for(int x = 0; x < 8; x++)
            {
                int index = x + 8 * y;
                newVal = context->readIntReg(index);
                if(regs[index] != newVal)
                {
                    sprintf(buf, " %s%d = 0x%016llx", prefixes[y], x, newVal);
                    outs << buf;
                    regs[index] = newVal;
                }
            }
        }
        for(int y = 0; y < 32; y++)
        {
            newVal = context->readFloatRegBits(2 * y, 64);
            if(floats[y] != newVal)
            {
                sprintf(buf, " F%d = 0x%016llx", y, newVal);
                outs << buf;
                floats[y] = newVal;
            }
        }
        outs << endl;
        /*
        int numSources = staticInst->numSrcRegs();
        int numDests = staticInst->numDestRegs();
        outs << "Sources:";
        for(int x = 0; x < numSources; x++)
        {
            int sourceNum = staticInst->srcRegIdx(x);
            if(sourceNum < FP_Base_DepTag)
                outs << " " << getIntRegName(sourceNum);
            else if(sourceNum < Ctrl_Base_DepTag)
                outs << " " << getFloatRegName(sourceNum - FP_Base_DepTag);
            else
                outs << " " << getMiscRegName(sourceNum - Ctrl_Base_DepTag);
        }
        outs << endl;
        outs << "Destinations:";
        for(int x = 0; x < numDests; x++)
        {
            int destNum = staticInst->destRegIdx(x);
            if(destNum < FP_Base_DepTag)
                outs << " " << getIntRegName(destNum);
            else if(destNum < Ctrl_Base_DepTag)
                outs << " " << getFloatRegName(destNum - FP_Base_DepTag);
            else
                outs << " " << getMiscRegName(destNum - Ctrl_Base_DepTag);
        }
        outs << endl;*/
    }
    else if (flags[INTEL_FORMAT]) {
#if FULL_SYSTEM
        bool is_trace_system = (cpu->system->name() == trace_system);
#else
        bool is_trace_system = true;
#endif
        if (is_trace_system) {
            ccprintf(outs, "%7d ) ", cycle);
            outs << "0x" << hex << PC << ":\t";
            if (staticInst->isLoad()) {
                outs << "<RD 0x" << hex << addr;
                outs << ">";
            } else if (staticInst->isStore()) {
                outs << "<WR 0x" << hex << addr;
                outs << ">";
            }
            outs << endl;
        }
    } else {
        if (flags[PRINT_CYCLE])
            ccprintf(outs, "%7d: ", cycle);

        outs << cpu->name() << " ";

        if (flags[TRACE_MISSPEC])
            outs << (misspeculating ? "-" : "+") << " ";

        if (flags[PRINT_THREAD_NUM])
            outs << "T" << thread << " : ";


        std::string sym_str;
        Addr sym_addr;
        if (debugSymbolTable
            && debugSymbolTable->findNearestSymbol(PC, sym_str, sym_addr)
            && flags[PC_SYMBOL]) {
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

#if defined(__GNUC__) && (__GNUC__ < 3)
        // There's a bug in gcc 2.x library that prevents setw()
        // from working properly on strings
        string mc(staticInst->disassemble(PC, debugSymbolTable));
        while (mc.length() < 26)
            mc += " ";
        outs << mc;
#else
        outs << setw(26) << left << staticInst->disassemble(PC, debugSymbolTable);
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
            for (int i = 0; i < TheISA::NumIntRegs;)
                for (int j = i + 1; i <= j; i++)
                    ccprintf(outs, "r%02d = %#018x%s", i,
                            iregs->regs.readReg(i),
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
    }
}


vector<bool> Trace::InstRecord::flags(NUM_BITS);
string Trace::InstRecord::trace_system;

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
                           "capture speculative instructions", true);

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
Param<bool> exe_trace_print_reg_delta(&exeTraceParams, "print_reg_delta",
                                  "print which registers changed to what", false);
Param<bool> exe_trace_pc_symbol(&exeTraceParams, "pc_symbol",
                                  "Use symbols for the PC if available", true);
Param<bool> exe_trace_intel_format(&exeTraceParams, "intel_format",
                                   "print trace in intel compatible format", false);
Param<string> exe_trace_system(&exeTraceParams, "trace_system",
                                   "print trace of which system (client or server)",
                                   "client");


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
    flags[PRINT_REG_DELTA]   = exe_trace_print_reg_delta;
    flags[PC_SYMBOL]         = exe_trace_pc_symbol;
    flags[INTEL_FORMAT]      = exe_trace_intel_format;
    trace_system	     = exe_trace_system;
}

void
ExecutionTraceParamContext::checkParams()
{
    Trace::InstRecord::setParams();
}

