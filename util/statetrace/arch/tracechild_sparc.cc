/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include <iostream>
#include <errno.h>
#include <sys/ptrace.h>
#include <stdint.h>

#include "tracechild_sparc.hh"

using namespace std;

string SparcTraceChild::regNames[numregs] = {
    //Global registers
    "g0", "g1", "g2", "g3", "g4", "g5", "g6", "g7",
    //Output registers
    "o0", "o1", "o2", "o3", "o4", "o5", "o6", "o7",
    //Local registers
    "l0", "l1", "l2", "l3", "l4", "l5", "l6", "l7",
    //Input registers
    "i0", "i1", "i2", "i3", "i4", "i5", "i6", "i7",
    //Floating point
    "f0", "f2", "f4", "f6", "f8", "f10", "f12", "f14",
    "f16", "f18", "f20", "f22", "f24", "f26", "f28", "f30",
    "f32", "f34", "f36", "f38", "f40", "f42", "f44", "f46",
    "f48", "f50", "f52", "f54", "f56", "f58", "f60", "f62",
    //Miscelaneous
    "fsr", "fprs", "pc", "npc", "y", "cwp", "pstate", "asi", "ccr"};

int64_t getRegs(regs & myregs, fpu & myfpu,
        int64_t * locals, int64_t * inputs, int num)
{
    assert(num < SparcTraceChild::numregs && num >= 0);
    switch(num)
    {
        //Global registers
        case SparcTraceChild::G0: return 0;
        case SparcTraceChild::G1: return myregs.r_g1;
        case SparcTraceChild::G2: return myregs.r_g2;
        case SparcTraceChild::G3: return myregs.r_g3;
        case SparcTraceChild::G4: return myregs.r_g4;
        case SparcTraceChild::G5: return myregs.r_g5;
        case SparcTraceChild::G6: return myregs.r_g6;
        case SparcTraceChild::G7: return myregs.r_g7;
        //Output registers
        case SparcTraceChild::O0: return myregs.r_o0;
        case SparcTraceChild::O1: return myregs.r_o1;
        case SparcTraceChild::O2: return myregs.r_o2;
        case SparcTraceChild::O3: return myregs.r_o3;
        case SparcTraceChild::O4: return myregs.r_o4;
        case SparcTraceChild::O5: return myregs.r_o5;
        case SparcTraceChild::O6: return myregs.r_o6;
        case SparcTraceChild::O7: return myregs.r_o7;
        //Local registers
        case SparcTraceChild::L0: return locals[0];
        case SparcTraceChild::L1: return locals[1];
        case SparcTraceChild::L2: return locals[2];
        case SparcTraceChild::L3: return locals[3];
        case SparcTraceChild::L4: return locals[4];
        case SparcTraceChild::L5: return locals[5];
        case SparcTraceChild::L6: return locals[6];
        case SparcTraceChild::L7: return locals[7];
        //Input registers
        case SparcTraceChild::I0: return inputs[0];
        case SparcTraceChild::I1: return inputs[1];
        case SparcTraceChild::I2: return inputs[2];
        case SparcTraceChild::I3: return inputs[3];
        case SparcTraceChild::I4: return inputs[4];
        case SparcTraceChild::I5: return inputs[5];
        case SparcTraceChild::I6: return inputs[6];
        case SparcTraceChild::I7: return inputs[7];
        //Floating point
        case SparcTraceChild::F0: return myfpu.f_fpstatus.fpu_fr[0];
        case SparcTraceChild::F2: return myfpu.f_fpstatus.fpu_fr[1];
        case SparcTraceChild::F4: return myfpu.f_fpstatus.fpu_fr[2];
        case SparcTraceChild::F6: return myfpu.f_fpstatus.fpu_fr[3];
        case SparcTraceChild::F8: return myfpu.f_fpstatus.fpu_fr[4];
        case SparcTraceChild::F10: return myfpu.f_fpstatus.fpu_fr[5];
        case SparcTraceChild::F12: return myfpu.f_fpstatus.fpu_fr[6];
        case SparcTraceChild::F14: return myfpu.f_fpstatus.fpu_fr[7];
        case SparcTraceChild::F16: return myfpu.f_fpstatus.fpu_fr[8];
        case SparcTraceChild::F18: return myfpu.f_fpstatus.fpu_fr[9];
        case SparcTraceChild::F20: return myfpu.f_fpstatus.fpu_fr[10];
        case SparcTraceChild::F22: return myfpu.f_fpstatus.fpu_fr[11];
        case SparcTraceChild::F24: return myfpu.f_fpstatus.fpu_fr[12];
        case SparcTraceChild::F26: return myfpu.f_fpstatus.fpu_fr[13];
        case SparcTraceChild::F28: return myfpu.f_fpstatus.fpu_fr[14];
        case SparcTraceChild::F30: return myfpu.f_fpstatus.fpu_fr[15];
        case SparcTraceChild::F32: return myfpu.f_fpstatus.fpu_fr[16];
        case SparcTraceChild::F34: return myfpu.f_fpstatus.fpu_fr[17];
        case SparcTraceChild::F36: return myfpu.f_fpstatus.fpu_fr[18];
        case SparcTraceChild::F38: return myfpu.f_fpstatus.fpu_fr[19];
        case SparcTraceChild::F40: return myfpu.f_fpstatus.fpu_fr[20];
        case SparcTraceChild::F42: return myfpu.f_fpstatus.fpu_fr[21];
        case SparcTraceChild::F44: return myfpu.f_fpstatus.fpu_fr[22];
        case SparcTraceChild::F46: return myfpu.f_fpstatus.fpu_fr[23];
        case SparcTraceChild::F48: return myfpu.f_fpstatus.fpu_fr[24];
        case SparcTraceChild::F50: return myfpu.f_fpstatus.fpu_fr[25];
        case SparcTraceChild::F52: return myfpu.f_fpstatus.fpu_fr[26];
        case SparcTraceChild::F54: return myfpu.f_fpstatus.fpu_fr[27];
        case SparcTraceChild::F56: return myfpu.f_fpstatus.fpu_fr[28];
        case SparcTraceChild::F58: return myfpu.f_fpstatus.fpu_fr[29];
        case SparcTraceChild::F60: return myfpu.f_fpstatus.fpu_fr[30];
        case SparcTraceChild::F62: return myfpu.f_fpstatus.fpu_fr[31];
        //Miscelaneous
        case SparcTraceChild::FSR: return myfpu.f_fpstatus.Fpu_fsr;
        case SparcTraceChild::FPRS: return myregs.r_fprs;
        case SparcTraceChild::PC: return myregs.r_tpc;
        case SparcTraceChild::NPC: return myregs.r_tnpc;
        case SparcTraceChild::Y: return myregs.r_y;
        case SparcTraceChild::CWP:
            return (myregs.r_tstate >> 0) & ((1 << 5) - 1);
        case SparcTraceChild::PSTATE:
            return (myregs.r_tstate >> 8) & ((1 << 13) - 1);
        case SparcTraceChild::ASI:
            return (myregs.r_tstate >> 24) & ((1 << 8) - 1);
        case SparcTraceChild::CCR:
            return (myregs.r_tstate >> 32) & ((1 << 8) - 1);
        default:
            assert(0);
            return 0;
    }
}

bool SparcTraceChild::update(int pid)
{
    static const int stackBias = 2047;
    memcpy(&oldregs, &theregs, sizeof(regs));
    memcpy(&oldfpregs, &thefpregs, sizeof(fpu));
    memcpy(oldLocals, locals, 8 * sizeof(uint64_t));
    memcpy(oldInputs, inputs, 8 * sizeof(uint64_t));
    if(ptrace(PTRACE_GETREGS, pid, &theregs, 0) != 0)
    {
        cerr << "Update failed" << endl;
        return false;
    }
    uint64_t StackPointer = getRegVal(O6);
    for(unsigned int x = 0; x < 8; x++)
    {
        locals[x] = ptrace(PTRACE_PEEKTEXT, pid,
            StackPointer + stackBias + x * 8, 0);
        inputs[x] = ptrace(PTRACE_PEEKTEXT, pid,
            StackPointer + stackBias + x * 8 + (8 * 8), 0);
    }
    if(ptrace(PTRACE_GETFPREGS, pid, &thefpregs, 0) != 0)
        return false;
    for(unsigned int x = 0; x < numregs; x++)
        regDiffSinceUpdate[x] = (getRegVal(x) != getOldRegVal(x));
    return true;
}

SparcTraceChild::SparcTraceChild()
{
    for(unsigned int x = 0; x < numregs; x++)
        regDiffSinceUpdate[x] = false;
}

bool SparcTraceChild::step()
{
    //Two important considerations are that the address of the instruction
    //being breakpointed should be word (64bit) aligned, and that both the
    //next instruction and the instruction after that need to be breakpointed
    //so that annulled branches will still stop as well.

    /*
     * Useful constants
     */
    const static uint64_t breakInst = 0x91d02001;
    const static uint64_t breakWord = breakInst | (breakInst << 32);
    const static uint64_t lowMask = 0xFFFFFFFFULL;
    const static uint64_t highMask = lowMask << 32;

    /*
     * storage for the original contents of the child process's memory
     */
    uint64_t originalInst, originalAnnulInst;

    /*
     * Get information about where the process is and is headed next.
     */
    uint64_t currentPC = getRegVal(PC);
    bool unalignedPC = currentPC & 7;
    uint64_t alignedPC = currentPC & (~7);
    uint64_t nextPC = getRegVal(NPC);
    bool unalignedNPC = nextPC & 7;
    uint64_t alignedNPC = nextPC & (~7);

    /*
     * Store the original contents of the child process's memory
     */
    originalInst = ptrace(PTRACE_PEEKTEXT, pid, alignedNPC, 0);
    //Save a ptrace call if we can
    if(unalignedNPC)
    {
        originalAnnulInst = ptrace(PTRACE_PEEKTEXT, pid, alignedNPC+8, 0);
    }

    /*
     * Prepare breakpointed copies of child processes memory
     */
    uint64_t newInst, newAnnulInst;
    //If the current instruction is in the same word as the npc
    if(alignedPC == alignedNPC)
    {
        //Make sure we only replace the other part
        if(unalignedPC)
            newInst = (originalInst & lowMask) | (breakWord & highMask);
        else
            newInst = (originalInst & highMask) | (breakWord & lowMask);
    }
    else
    {
        //otherwise replace the whole thing
        newInst = breakWord;
    }
    //If the current instruction is in the same word as the word after
    //the npc
    if(alignedPC == alignedNPC+8)
    {
        //Make sure we only replace the other part
        if(unalignedPC)
            newAnnulInst = (originalAnnulInst & lowMask) | (breakWord & highMask);
        else
            newAnnulInst = (originalAnnulInst & highMask) | (breakWord & lowMask);
    }
    else
    {
        //otherwise replace the whole thing
        newAnnulInst = breakWord;
    }

    /*
     * Stuff the breakpoint instructions into the child's address space.
     */
    //Replace the word at npc
    if(ptrace(PTRACE_POKETEXT, pid, alignedNPC, newInst) != 0)
        cerr << "Poke failed" << endl;
    //Replace the next word, if necessary
    if(unalignedNPC)
    {
        if(ptrace(PTRACE_POKETEXT, pid, alignedNPC+8, newAnnulInst) != 0)
            cerr << "Poke failed" << endl;
    }

    /*
     * Restart the child process
     */
    //Note that the "addr" parameter is supposed to be ignored, but in at
    //least one version of the kernel, it must be 1 or it will set what
    //pc to continue from
    if(ptrace(PTRACE_CONT, pid, 1, 0) != 0)
        cerr << "Cont failed" << endl;
    doWait();

    /*
     * Update our record of the child's state
     */
    update(pid);

    /*
     * Put back the original contents of the childs address space
     */
    if(ptrace(PTRACE_POKETEXT, pid, alignedNPC, originalInst) != 0)
        cerr << "Repoke failed" << endl;
    if(unalignedNPC)
    {
        if(ptrace(PTRACE_POKETEXT, pid, alignedNPC+8, originalAnnulInst) != 0)
            cerr << "Repoke failed" << endl;
    }
    return true;
}

int64_t SparcTraceChild::getRegVal(int num)
{
    return getRegs(theregs, thefpregs, locals, inputs, num);
}

int64_t SparcTraceChild::getOldRegVal(int num)
{
    return getRegs(oldregs, oldfpregs, oldLocals, oldInputs, num);
}

char * SparcTraceChild::printReg(int num)
{
    sprintf(printBuffer, "0x%016llx", getRegVal(num));
    return printBuffer;
}

ostream & SparcTraceChild::outputStartState(ostream & os)
{
    uint64_t sp = getSP();
    uint64_t pc = getPC();
    char obuf[1024];
    sprintf(obuf, "Initial stack pointer = 0x%016llx\n", sp);
    os << obuf;
    sprintf(obuf, "Initial program counter = 0x%016llx\n", pc);
    os << obuf;
    //Take out the stack bias
    sp += 2047;
    //Output the window save area
    for(unsigned int x = 0; x < 16; x++)
    {
        uint64_t regspot = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%016llx: Window save %d = 0x%016llx\n",
                sp, x+1, regspot);
        os << obuf;
        sp += 8;
    }
    //Output the argument count
    uint64_t cargc = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
    sprintf(obuf, "0x%016llx: Argc = 0x%016llx\n", sp, cargc);
    os << obuf;
    sp += 8;
    //Output argv pointers
    int argCount = 0;
    uint64_t cargv;
    do
    {
        cargv = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%016llx: argv[%d] = 0x%016llx\n",
                sp, argCount++, cargv);
        os << obuf;
        sp += 8;
    } while(cargv);
    //Output the envp pointers
    int envCount = 0;
    uint64_t cenvp;
    do
    {
        cenvp = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%016llx: envp[%d] = 0x%016llx\n",
                sp, envCount++, cenvp);
        os << obuf;
        sp += 8;
    } while(cenvp);
    uint64_t auxType, auxVal;
    do
    {
        auxType = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sp += 8;
        auxVal = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sp += 8;
        sprintf(obuf, "0x%016llx: Auxiliary vector = {0x%016llx, 0x%016llx}\n",
                sp - 16, auxType, auxVal);
        os << obuf;
    } while(auxType != 0 || auxVal != 0);
    //Print out the argument strings, environment strings, and file name.
    string current;
    uint64_t buf;
    uint64_t currentStart = sp;
    bool clearedInitialPadding = false;
    do
    {
        buf = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        char * cbuf = (char *)&buf;
        for(int x = 0; x < sizeof(uint64_t); x++)
        {
            if(cbuf[x])
                current += cbuf[x];
            else
            {
                sprintf(obuf, "0x%016llx: \"%s\"\n",
                        currentStart, current.c_str());
                os << obuf;
                current = "";
                currentStart = sp + x + 1;
            }
        }
        sp += 8;
        clearedInitialPadding = clearedInitialPadding || buf != 0;
    } while(!clearedInitialPadding || buf != 0);
    return os;
}

TraceChild * genTraceChild()
{
    return new SparcTraceChild;
}

