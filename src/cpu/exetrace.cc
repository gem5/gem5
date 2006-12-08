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
#include <sys/ipc.h>
#include <sys/shm.h>

#include "arch/regfile.hh"
#include "arch/utility.hh"
#include "base/loader/symtab.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/exetrace.hh"
#include "cpu/static_inst.hh"
#include "sim/param.hh"
#include "sim/system.hh"

#if FULL_SYSTEM
#include "arch/tlb.hh"
#endif

//XXX This is temporary
#include "arch/isa_specific.hh"
#include "cpu/m5legion_interface.h"

using namespace std;
using namespace TheISA;

namespace Trace {
SharedData *shared_data = NULL;
}

////////////////////////////////////////////////////////////////////////
//
//  Methods for the InstRecord object
//

#if THE_ISA == SPARC_ISA

inline char * genCenteredLabel(int length, char * buffer, char * label)
{
    int labelLength = strlen(label);
    assert(labelLength <= length);
    int leftPad = (length - labelLength) / 2;
    int rightPad = length - leftPad - labelLength;
    char format[64];
    sprintf(format, "%%%ds%%s%%%ds", leftPad, rightPad);
    sprintf(buffer, format, "", label, "");
    return buffer;
}

inline void printRegPair(ostream & os, char const * title, uint64_t a, uint64_t b)
{
    ccprintf(os, "  %16s  |  %#018x   %s   %#-018x  \n",
            title, a, (a == b) ? "|" : "X", b);
}

inline void printColumnLabels(ostream & os)
{
    static char * regLabel = genCenteredLabel(16, new char[17], "Register");
    static char * m5Label = genCenteredLabel(18, new char[18], "M5");
    static char * legionLabel = genCenteredLabel(18, new char[18], "Legion");
    ccprintf(os, "  %s  |  %s   |   %s  \n", regLabel, m5Label, legionLabel);
    ccprintf(os, "--------------------+-----------------------+-----------------------\n");
}

inline void printSectionHeader(ostream & os, char * name)
{
    char sectionString[70];
    genCenteredLabel(69, sectionString, name);
    ccprintf(os, "====================================================================\n");
    ccprintf(os, "%69s\n", sectionString);
    ccprintf(os, "====================================================================\n");
}

inline void printLevelHeader(ostream & os, int level)
{
    char sectionString[70];
    char levelName[70];
    sprintf(levelName, "Trap stack level %d", level);
    genCenteredLabel(69, sectionString, levelName);
    ccprintf(os, "====================================================================\n");
    ccprintf(os, "%69s\n", sectionString);
    ccprintf(os, "====================================================================\n");
}

#endif

void
Trace::InstRecord::dump(ostream &outs)
{
    if (flags[PRINT_REG_DELTA])
    {
#if THE_ISA == SPARC_ISA
        //Don't print what happens for each micro-op, just print out
        //once at the last op, and for regular instructions.
        if(!staticInst->isMicroOp() || staticInst->isLastMicroOp())
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

            outs << hex;
            outs << "PC = " << thread->readNextPC();
            outs << " NPC = " << thread->readNextNPC();
            newVal = thread->readIntReg(SparcISA::NumIntArchRegs + 2);
            //newVal = thread->readMiscReg(SparcISA::MISCREG_CCR);
            if(newVal != ccr)
            {
                outs << " CCR = " << newVal;
                ccr = newVal;
            }
            newVal = thread->readIntReg(SparcISA::NumIntArchRegs + 1);
            //newVal = thread->readMiscReg(SparcISA::MISCREG_Y);
            if(newVal != y)
            {
                outs << " Y = " << newVal;
                y = newVal;
            }
            for(int y = 0; y < 4; y++)
            {
                for(int x = 0; x < 8; x++)
                {
                    int index = x + 8 * y;
                    newVal = thread->readIntReg(index);
                    if(regs[index] != newVal)
                    {
                        outs << " " << prefixes[y] << dec << x << " = " << hex << newVal;
                        regs[index] = newVal;
                    }
                }
            }
            for(int y = 0; y < 32; y++)
            {
                newVal = thread->readFloatRegBits(2 * y, 64);
                if(floats[y] != newVal)
                {
                    outs << " F" << dec << (2 * y) << " = " << hex << newVal;
                    floats[y] = newVal;
                }
            }
            outs << dec << endl;
        }
#endif
    }
    else if (flags[INTEL_FORMAT]) {
#if FULL_SYSTEM
        bool is_trace_system = (thread->getCpuPtr()->system->name() == trace_system);
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

        outs << thread->getCpuPtr()->name() << " ";

        if (flags[TRACE_MISSPEC])
            outs << (misspeculating ? "-" : "+") << " ";

        if (flags[PRINT_THREAD_NUM])
            outs << "T" << thread->getThreadNum() << " : ";


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
#if THE_ISA == SPARC_ISA
    // Compare
    if (flags[LEGION_LOCKSTEP])
    {
        bool compared = false;
        bool diffPC   = false;
        bool diffInst = false;
        bool diffRegs = false;
        bool diffTpc = false;
        bool diffTnpc = false;
        bool diffTstate = false;
        bool diffTt = false;
        bool diffTba = false;
        bool diffHpstate = false;
        bool diffHtstate = false;
        bool diffHtba = false;
        bool diffPstate = false;
        bool diffY = false;
        bool diffCcr = false;
        bool diffTl = false;
        bool diffGl = false;
        bool diffAsi = false;
        bool diffPil = false;
        bool diffCwp = false;
        bool diffCansave = false;
        bool diffCanrestore = false;
        bool diffOtherwin = false;
        bool diffCleanwin = false;
        Addr m5Pc, lgnPc;


        if(!staticInst->isMicroOp() || staticInst->isLastMicroOp()) {
            while (!compared) {
                if (shared_data->flags == OWN_M5) {
                    m5Pc = PC & TheISA::PAddrImplMask;
                    lgnPc = shared_data->pc & TheISA::PAddrImplMask;
                    if (lgnPc != m5Pc)
                       diffPC = true;
                    if (shared_data->instruction !=
                            (SparcISA::MachInst)staticInst->machInst) {
                        diffInst = true;
                    }
                    for (int i = 0; i < TheISA::NumIntArchRegs; i++) {
                        if (thread->readIntReg(i) != shared_data->intregs[i]) {
                            diffRegs = true;
                        }
                    }
                    uint64_t oldTl = thread->readMiscReg(MISCREG_TL);
                    if (oldTl != shared_data->tl)
                        diffTl = true;
                    for (int i = 1; i <= MaxTL; i++) {
                        thread->setMiscReg(MISCREG_TL, i);
                        if (thread->readMiscReg(MISCREG_TPC) !=
                                shared_data->tpc[i])
                            diffTpc = true;
                        if (thread->readMiscReg(MISCREG_TNPC) !=
                                shared_data->tnpc[i])
                            diffTnpc = true;
                        if (thread->readMiscReg(MISCREG_TSTATE) !=
                                shared_data->tstate[i])
                            diffTstate = true;
                        if (thread->readMiscReg(MISCREG_TT) !=
                                shared_data->tt[i])
                            diffTt = true;
                        if (thread->readMiscReg(MISCREG_HTSTATE) !=
                                shared_data->htstate[i])
                            diffHtstate = true;
                    }
                    thread->setMiscReg(MISCREG_TL, oldTl);

                    if(shared_data->tba != thread->readMiscReg(MISCREG_TBA))
                        diffTba = true;
                    //When the hpstate register is read by an instruction,
                    //legion has bit 11 set. When it's in storage, it doesn't.
                    //Since we don't directly support seperate interpretations
                    //of the registers like that, the bit is always set to 1 and
                    //we just don't compare it. It's not supposed to matter
                    //anyway.
                    if((shared_data->hpstate | (1 << 11)) != thread->readMiscReg(MISCREG_HPSTATE))
                        diffHpstate = true;
                    if(shared_data->htba != thread->readMiscReg(MISCREG_HTBA))
                        diffHtba = true;
                    if(shared_data->pstate != thread->readMiscReg(MISCREG_PSTATE))
                        diffPstate = true;
                    //if(shared_data->y != thread->readMiscReg(MISCREG_Y))
                    if(shared_data->y !=
                            thread->readIntReg(NumIntArchRegs + 1))
                        diffY = true;
                    //if(shared_data->ccr != thread->readMiscReg(MISCREG_CCR))
                    if(shared_data->ccr !=
                            thread->readIntReg(NumIntArchRegs + 2))
                        diffCcr = true;
                    if(shared_data->gl != thread->readMiscReg(MISCREG_GL))
                        diffGl = true;
                    if(shared_data->asi != thread->readMiscReg(MISCREG_ASI))
                        diffAsi = true;
                    if(shared_data->pil != thread->readMiscReg(MISCREG_PIL))
                        diffPil = true;
                    if(shared_data->cwp != thread->readMiscReg(MISCREG_CWP))
                        diffCwp = true;
                    //if(shared_data->cansave != thread->readMiscReg(MISCREG_CANSAVE))
                    if(shared_data->cansave !=
                            thread->readIntReg(NumIntArchRegs + 3))
                        diffCansave = true;
                    //if(shared_data->canrestore !=
                    //	    thread->readMiscReg(MISCREG_CANRESTORE))
                    if(shared_data->canrestore !=
                            thread->readMiscReg(NumIntArchRegs + 4))
                        diffCanrestore = true;
                    //if(shared_data->otherwin != thread->readMiscReg(MISCREG_OTHERWIN))
                    if(shared_data->otherwin !=
                            thread->readIntReg(NumIntArchRegs + 5))
                        diffOtherwin = true;
                    //if(shared_data->cleanwin != thread->readMiscReg(MISCREG_CLEANWIN))
                    if(shared_data->cleanwin !=
                            thread->readMiscReg(NumIntArchRegs + 6))
                        diffCleanwin = true;

                    if (diffPC || diffInst || diffRegs || diffTpc || diffTnpc ||
                            diffTstate || diffTt || diffHpstate ||
                            diffHtstate || diffHtba || diffPstate || diffY ||
                            diffCcr || diffTl || diffGl || diffAsi || diffPil ||
                            diffCwp || diffCansave || diffCanrestore ||
                            diffOtherwin || diffCleanwin) {
                        outs << "Differences found between M5 and Legion:";
                        if (diffPC)
                            outs << " [PC]";
                        if (diffInst)
                            outs << " [Instruction]";
                        if (diffRegs)
                            outs << " [IntRegs]";
                        if (diffTpc)
                            outs << " [Tpc]";
                        if (diffTnpc)
                            outs << " [Tnpc]";
                        if (diffTstate)
                            outs << " [Tstate]";
                        if (diffTt)
                            outs << " [Tt]";
                        if (diffHpstate)
                            outs << " [Hpstate]";
                        if (diffHtstate)
                            outs << " [Htstate]";
                        if (diffHtba)
                            outs << " [Htba]";
                        if (diffPstate)
                            outs << " [Pstate]";
                        if (diffY)
                            outs << " [Y]";
                        if (diffCcr)
                            outs << " [Ccr]";
                        if (diffTl)
                            outs << " [Tl]";
                        if (diffGl)
                            outs << " [Gl]";
                        if (diffAsi)
                            outs << " [Asi]";
                        if (diffPil)
                            outs << " [Pil]";
                        if (diffCwp)
                            outs << " [Cwp]";
                        if (diffCansave)
                            outs << " [Cansave]";
                        if (diffCanrestore)
                            outs << " [Canrestore]";
                        if (diffOtherwin)
                            outs << " [Otherwin]";
                        if (diffCleanwin)
                            outs << " [Cleanwin]";
                        outs << endl << endl;

                        outs << right << setfill(' ') << setw(15)
                             << "M5 PC: " << "0x"<< setw(16) << setfill('0')
                             << hex << m5Pc << endl;
                        outs << setfill(' ') << setw(15)
                             << "Legion PC: " << "0x"<< setw(16) << setfill('0') << hex
                             << lgnPc << endl << endl;

                        outs << setfill(' ') << setw(15)
                             << "M5 Inst: "  << "0x"<< setw(8)
                             << setfill('0') << hex << staticInst->machInst
                             << staticInst->disassemble(m5Pc, debugSymbolTable)
                             << endl;

                        StaticInstPtr legionInst =
                            StaticInst::decode(makeExtMI(shared_data->instruction,
                                        thread));
                        outs << setfill(' ') << setw(15)
                             << " Legion Inst: "
                             << "0x" << setw(8) << setfill('0') << hex
                             << shared_data->instruction
                             << legionInst->disassemble(lgnPc, debugSymbolTable)
                             << endl << endl;

                        printSectionHeader(outs, "General State");
                        printColumnLabels(outs);
                        printRegPair(outs, "HPstate",
                                thread->readMiscReg(MISCREG_HPSTATE),
                                shared_data->hpstate | (1 << 11));
                        printRegPair(outs, "Htba",
                                thread->readMiscReg(MISCREG_HTBA),
                                shared_data->htba);
                        printRegPair(outs, "Pstate",
                                thread->readMiscReg(MISCREG_PSTATE),
                                shared_data->pstate);
                        printRegPair(outs, "Y",
                                //thread->readMiscReg(MISCREG_Y),
                                thread->readMiscReg(NumIntArchRegs + 1),
                                shared_data->y);
                        printRegPair(outs, "Ccr",
                                //thread->readMiscReg(MISCREG_CCR),
                                thread->readMiscReg(NumIntArchRegs + 2),
                                shared_data->ccr);
                        printRegPair(outs, "Tl",
                                thread->readMiscReg(MISCREG_TL),
                                shared_data->tl);
                        printRegPair(outs, "Gl",
                                thread->readMiscReg(MISCREG_GL),
                                shared_data->gl);
                        printRegPair(outs, "Asi",
                                thread->readMiscReg(MISCREG_ASI),
                                shared_data->asi);
                        printRegPair(outs, "Pil",
                                thread->readMiscReg(MISCREG_PIL),
                                shared_data->pil);
                        printRegPair(outs, "Cwp",
                                thread->readMiscReg(MISCREG_CWP),
                                shared_data->cwp);
                        printRegPair(outs, "Cansave",
                                //thread->readMiscReg(MISCREG_CANSAVE),
                                thread->readIntReg(NumIntArchRegs + 3),
                                shared_data->cansave);
                        printRegPair(outs, "Canrestore",
                                //thread->readMiscReg(MISCREG_CANRESTORE),
                                thread->readIntReg(NumIntArchRegs + 4),
                                shared_data->canrestore);
                        printRegPair(outs, "Otherwin",
                                //thread->readMiscReg(MISCREG_OTHERWIN),
                                thread->readIntReg(NumIntArchRegs + 5),
                                shared_data->otherwin);
                        printRegPair(outs, "Cleanwin",
                                //thread->readMiscReg(MISCREG_CLEANWIN),
                                thread->readIntReg(NumIntArchRegs + 6),
                                shared_data->cleanwin);
                        outs << endl;
                        for (int i = 1; i <= MaxTL; i++) {
                            printLevelHeader(outs, i);
                            printColumnLabels(outs);
                            thread->setMiscReg(MISCREG_TL, i);
                            printRegPair(outs, "Tpc",
                                    thread->readMiscReg(MISCREG_TPC),
                                    shared_data->tpc[i]);
                            printRegPair(outs, "Tnpc",
                                    thread->readMiscReg(MISCREG_TNPC),
                                    shared_data->tnpc[i]);
                            printRegPair(outs, "Tstate",
                                    thread->readMiscReg(MISCREG_TSTATE),
                                    shared_data->tstate[i]);
                            printRegPair(outs, "Tt",
                                    thread->readMiscReg(MISCREG_TT),
                                    shared_data->tt[i]);
                            printRegPair(outs, "Htstate",
                                    thread->readMiscReg(MISCREG_HTSTATE),
                                    shared_data->htstate[i]);
                        }
                        thread->setMiscReg(MISCREG_TL, oldTl);
                        outs << endl;

                        printSectionHeader(outs, "General Purpose Registers");
                        static const char * regtypes[4] = {"%g", "%o", "%l", "%i"};
                        for(int y = 0; y < 4; y++)
                        {
                            for(int x = 0; x < 8; x++)
                            {
                                char label[8];
                                sprintf(label, "%s%d", regtypes[y], x);
                                printRegPair(outs, label,
                                        thread->readIntReg(y*8+x),
                                        shared_data->intregs[y*8+x]);
                                /*outs << regtypes[y] << x << "         " ;
                                outs <<  "0x" << hex << setw(16)
                                    << thread->readIntReg(y*8+x);
                                if (thread->readIntReg(y*8 + x)
                                        != shared_data->intregs[y*8+x])
                                    outs << "     X     ";
                                else
                                    outs << "     |     ";
                                outs << "0x" << setw(16) << hex
                                    << shared_data->intregs[y*8+x]
                                    << endl;*/
                            }
                        }
                        fatal("Differences found between Legion and M5\n");
                    }

                    compared = true;
                    shared_data->flags = OWN_LEGION;
                }
            } // while
        } // if not microop
    }
#endif
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
Param<bool> exe_trace_legion_lockstep(&exeTraceParams, "legion_lockstep",
                                   "Compare sim state to legion state every cycle",
                                   false);
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
    flags[LEGION_LOCKSTEP]   = exe_trace_legion_lockstep;
    trace_system	     = exe_trace_system;

    // If were going to be in lockstep with Legion
    // Setup shared memory, and get otherwise ready
    if (flags[LEGION_LOCKSTEP]) {
        int shmfd = shmget('M' << 24 | getuid(), sizeof(SharedData), 0777);
        if (shmfd < 0)
            fatal("Couldn't get shared memory fd. Is Legion running?");

        shared_data = (SharedData*)shmat(shmfd, NULL, SHM_RND);
        if (shared_data == (SharedData*)-1)
            fatal("Couldn't allocate shared memory");

        if (shared_data->flags != OWN_M5)
            fatal("Shared memory has invalid owner");

        if (shared_data->version != VERSION)
            fatal("Shared Data is wrong version! M5: %d Legion: %d", VERSION,
                    shared_data->version);

        // step legion forward one cycle so we can get register values
        shared_data->flags = OWN_LEGION;
    }
}

void
ExecutionTraceParamContext::checkParams()
{
    Trace::InstRecord::setParams();
}

