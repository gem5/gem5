/*
 * Copyright (c) 2007 The Regents of The University of Michigan
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

#include <sys/ptrace.h>
#include <stdint.h>

#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "arch/amd64/tracechild.hh"

using namespace std;

bool
AMD64TraceChild::sendState(int socket)
{
    uint64_t regVal64 = 0;
    uint32_t regVal32 = 0;
    for (int x = 0; x <= R15; x++) {
        regVal64 = getRegVal(x);
        if (write(socket, &regVal64, sizeof(regVal64)) == -1) {
            cerr << "Write failed! " << strerror(errno) << endl;
            tracing = false;
            return false;
        }
    }
    regVal64 = getRegVal(RIP);
    if (write(socket, &regVal64, sizeof(regVal64)) == -1) {
        cerr << "Write failed! " << strerror(errno) << endl;
        tracing = false;
        return false;
    }
    for (int x = MMX0_0; x <= MMX7_1; x++) {
        regVal32 = getRegVal(x);
        if (write(socket, &regVal32, sizeof(regVal32)) == -1) {
            cerr << "Write failed! " << strerror(errno) << endl;
            tracing = false;
            return false;
        }
    }
    for (int x = XMM0_0; x <= XMM15_3; x++) {
        regVal32 = getRegVal(x);
        if (write(socket, &regVal32, sizeof(regVal32)) == -1) {
            cerr << "Write failed! " << strerror(errno) << endl;
            tracing = false;
            return false;
        }
    }
    return true;
}

int64_t
AMD64TraceChild::getRegs(user_regs_struct &myregs,
                         user_fpregs_struct &myfpregs, int num)
{
    assert(num < numregs && num >= 0);
    switch (num) {
    // GPRs
    case RAX:
        return myregs.rax;
    case RBX:
        return myregs.rbx;
    case RCX:
        return myregs.rcx;
    case RDX:
        return myregs.rdx;
    // Index registers
    case RSI:
        return myregs.rsi;
    case RDI:
        return myregs.rdi;
    // Base pointer and stack pointer
    case RBP:
        return myregs.rbp;
    case RSP:
        return myregs.rsp;
    // New 64 bit mode registers
    case R8:
        return myregs.r8;
    case R9:
        return myregs.r9;
    case R10:
        return myregs.r10;
    case R11:
        return myregs.r11;
    case R12:
        return myregs.r12;
    case R13:
        return myregs.r13;
    case R14:
        return myregs.r14;
    case R15:
        return myregs.r15;
    // Segmentation registers
    case CS:
        return myregs.cs;
    case DS:
        return myregs.ds;
    case ES:
        return myregs.es;
    case FS:
        return myregs.fs;
    case GS:
        return myregs.gs;
    case SS:
        return myregs.ss;
    case FS_BASE:
        return myregs.fs_base;
    case GS_BASE:
        return myregs.gs_base;
    // PC
    case RIP:
        return myregs.rip;
    // Flags
    case EFLAGS:
        return myregs.eflags;
    // MMX
    case MMX0_0:
        return myfpregs.st_space[0];
    case MMX0_1:
        return myfpregs.st_space[1];
    case MMX1_0:
        return myfpregs.st_space[2];
    case MMX1_1:
        return myfpregs.st_space[3];
    case MMX2_0:
        return myfpregs.st_space[4];
    case MMX2_1:
        return myfpregs.st_space[5];
    case MMX3_0:
        return myfpregs.st_space[6];
    case MMX3_1:
        return myfpregs.st_space[7];
    case MMX4_0:
        return myfpregs.st_space[8];
    case MMX4_1:
        return myfpregs.st_space[9];
    case MMX5_0:
        return myfpregs.st_space[10];
    case MMX5_1:
        return myfpregs.st_space[11];
    case MMX6_0:
        return myfpregs.st_space[12];
    case MMX6_1:
        return myfpregs.st_space[13];
    case MMX7_0:
        return myfpregs.st_space[14];
    case MMX7_1:
        return myfpregs.st_space[15];
    // XMM
    case XMM0_0:
        return myfpregs.xmm_space[0];
    case XMM0_1:
        return myfpregs.xmm_space[1];
    case XMM0_2:
        return myfpregs.xmm_space[2];
    case XMM0_3:
        return myfpregs.xmm_space[3];
    case XMM1_0:
        return myfpregs.xmm_space[4];
    case XMM1_1:
        return myfpregs.xmm_space[5];
    case XMM1_2:
        return myfpregs.xmm_space[6];
    case XMM1_3:
        return myfpregs.xmm_space[7];
    case XMM2_0:
        return myfpregs.xmm_space[8];
    case XMM2_1:
        return myfpregs.xmm_space[9];
    case XMM2_2:
        return myfpregs.xmm_space[10];
    case XMM2_3:
        return myfpregs.xmm_space[11];
    case XMM3_0:
        return myfpregs.xmm_space[12];
    case XMM3_1:
        return myfpregs.xmm_space[13];
    case XMM3_2:
        return myfpregs.xmm_space[14];
    case XMM3_3:
        return myfpregs.xmm_space[15];
    case XMM4_0:
        return myfpregs.xmm_space[16];
    case XMM4_1:
        return myfpregs.xmm_space[17];
    case XMM4_2:
        return myfpregs.xmm_space[18];
    case XMM4_3:
        return myfpregs.xmm_space[19];
    case XMM5_0:
        return myfpregs.xmm_space[20];
    case XMM5_1:
        return myfpregs.xmm_space[21];
    case XMM5_2:
        return myfpregs.xmm_space[22];
    case XMM5_3:
        return myfpregs.xmm_space[23];
    case XMM6_0:
        return myfpregs.xmm_space[24];
    case XMM6_1:
        return myfpregs.xmm_space[25];
    case XMM6_2:
        return myfpregs.xmm_space[26];
    case XMM6_3:
        return myfpregs.xmm_space[27];
    case XMM7_0:
        return myfpregs.xmm_space[28];
    case XMM7_1:
        return myfpregs.xmm_space[29];
    case XMM7_2:
        return myfpregs.xmm_space[30];
    case XMM7_3:
        return myfpregs.xmm_space[31];
    case XMM8_0:
        return myfpregs.xmm_space[32];
    case XMM8_1:
        return myfpregs.xmm_space[33];
    case XMM8_2:
        return myfpregs.xmm_space[34];
    case XMM8_3:
        return myfpregs.xmm_space[35];
    case XMM9_0:
        return myfpregs.xmm_space[36];
    case XMM9_1:
        return myfpregs.xmm_space[37];
    case XMM9_2:
        return myfpregs.xmm_space[38];
    case XMM9_3:
        return myfpregs.xmm_space[39];
    case XMM10_0:
        return myfpregs.xmm_space[40];
    case XMM10_1:
        return myfpregs.xmm_space[41];
    case XMM10_2:
        return myfpregs.xmm_space[42];
    case XMM10_3:
        return myfpregs.xmm_space[43];
    case XMM11_0:
        return myfpregs.xmm_space[44];
    case XMM11_1:
        return myfpregs.xmm_space[45];
    case XMM11_2:
        return myfpregs.xmm_space[46];
    case XMM11_3:
        return myfpregs.xmm_space[47];
    case XMM12_0:
        return myfpregs.xmm_space[48];
    case XMM12_1:
        return myfpregs.xmm_space[49];
    case XMM12_2:
        return myfpregs.xmm_space[50];
    case XMM12_3:
        return myfpregs.xmm_space[51];
    case XMM13_0:
        return myfpregs.xmm_space[52];
    case XMM13_1:
        return myfpregs.xmm_space[53];
    case XMM13_2:
        return myfpregs.xmm_space[54];
    case XMM13_3:
        return myfpregs.xmm_space[55];
    case XMM14_0:
        return myfpregs.xmm_space[56];
    case XMM14_1:
        return myfpregs.xmm_space[57];
    case XMM14_2:
        return myfpregs.xmm_space[58];
    case XMM14_3:
        return myfpregs.xmm_space[59];
    case XMM15_0:
        return myfpregs.xmm_space[60];
    case XMM15_1:
        return myfpregs.xmm_space[61];
    case XMM15_2:
        return myfpregs.xmm_space[62];
    case XMM15_3:
        return myfpregs.xmm_space[63];
    default:
        assert(0);
        return 0;
    }
}

bool
AMD64TraceChild::update(int pid)
{
    oldregs = regs;
    oldfpregs = fpregs;
    if (ptrace(PTRACE_GETREGS, pid, 0, &regs) != 0) {
        cerr << "update: " << strerror(errno) << endl;
        return false;
    }
    if (ptrace(PTRACE_GETFPREGS, pid, 0, &fpregs) != 0) {
        cerr << "update: " << strerror(errno) << endl;
        return false;
    }
    for (unsigned int x = 0; x < numregs; x++)
        regDiffSinceUpdate[x] = (getRegVal(x) != getOldRegVal(x));
    return true;
}

AMD64TraceChild::AMD64TraceChild()
{
    for (unsigned int x = 0; x < numregs; x++)
        regDiffSinceUpdate[x] = false;
}

int64_t
AMD64TraceChild::getRegVal(int num)
{
    return getRegs(regs, fpregs, num);
}

int64_t
AMD64TraceChild::getOldRegVal(int num)
{
    return getRegs(oldregs, oldfpregs, num);
}

ostream &
AMD64TraceChild::outputStartState(ostream &os)
{
    uint64_t sp = getSP();
    uint64_t pc = getPC();
    uint64_t highestInfo = 0;
    char obuf[1024];
    sprintf(obuf, "Initial stack pointer = 0x%016lx\n", sp);
    os << obuf;
    sprintf(obuf, "Initial program counter = 0x%016lx\n", pc);
    os << obuf;

    // Output the argument count
    uint64_t cargc = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
    sprintf(obuf, "0x%016lx: Argc = 0x%016lx\n", sp, cargc);
    os << obuf;
    sp += 8;

    // Output argv pointers
    int argCount = 0;
    uint64_t cargv;
    do {
        cargv = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%016lx: argv[%d] = 0x%016lx\n", sp, argCount++,
                cargv);
        if (cargv)
            if (highestInfo < cargv)
                highestInfo = cargv;
        os << obuf;
        sp += 8;
    } while (cargv);

    // Output the envp pointers
    int envCount = 0;
    uint64_t cenvp;
    do {
        cenvp = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sprintf(obuf, "0x%016lx: envp[%d] = 0x%016lx\n", sp, envCount++,
                cenvp);
        os << obuf;
        sp += 8;
    } while (cenvp);
    uint64_t auxType, auxVal;
    do {
        auxType = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sp += 8;
        auxVal = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        sp += 8;
        sprintf(obuf, "0x%016lx: Auxiliary vector = {0x%016lx, 0x%016lx}\n",
                sp - 16, auxType, auxVal);
        os << obuf;
    } while (auxType != 0 || auxVal != 0);
    // Print out the argument strings, environment strings, and file name.
    string current;
    uint64_t buf;
    uint64_t currentStart = sp;
    bool clearedInitialPadding = false;
    do {
        buf = ptrace(PTRACE_PEEKDATA, pid, sp, 0);
        char *cbuf = (char *)&buf;
        for (int x = 0; x < sizeof(uint64_t); x++) {
            if (cbuf[x])
                current += cbuf[x];
            else {
                sprintf(obuf, "0x%016lx: \"%s\"\n", currentStart,
                        current.c_str());
                os << obuf;
                current = "";
                currentStart = sp + x + 1;
            }
        }
        sp += 8;
        clearedInitialPadding = clearedInitialPadding || buf != 0;
    } while (!clearedInitialPadding || buf != 0 || sp <= highestInfo);
    return os;
}

uint64_t
AMD64TraceChild::findSyscall()
{
    uint64_t rip = getPC();
    bool foundOpcode = false;
    bool twoByteOpcode = false;
    for (;;) {
        uint64_t buf = ptrace(PTRACE_PEEKDATA, pid, rip, 0);
        for (int i = 0; i < sizeof(uint64_t); i++) {
            unsigned char byte = buf & 0xFF;
            if (!foundOpcode) {
                if (!(byte == 0x66 ||                // operand override
                      byte == 0x67 ||                // address override
                      byte == 0x2E ||                // cs
                      byte == 0x3E ||                // ds
                      byte == 0x26 ||                // es
                      byte == 0x64 ||                // fs
                      byte == 0x65 ||                // gs
                      byte == 0x36 ||                // ss
                      byte == 0xF0 ||                // lock
                      byte == 0xF2 ||                // repe
                      byte == 0xF3 ||                // repne
                      (byte >= 0x40 && byte <= 0x4F) // REX
                      )) {
                    foundOpcode = true;
                }
            }
            if (foundOpcode) {
                if (twoByteOpcode) {
                    // SYSCALL or SYSENTER
                    if (byte == 0x05 || byte == 0x34)
                        return rip + 1;
                    else
                        return 0;
                }
                if (!twoByteOpcode) {
                    if (byte == 0xCC) // INT3
                        return rip + 1;
                    else if (byte == 0xCD) // INT with byte immediate
                        return rip + 2;
                    else if (byte == 0x0F) // two byte opcode prefix
                        twoByteOpcode = true;
                    else
                        return 0;
                }
            }
            buf >>= 8;
            rip++;
        }
    }
}

bool
AMD64TraceChild::step()
{
    uint64_t ripAfterSyscall = findSyscall();
    if (ripAfterSyscall) {
        // Get the original contents of memory
        uint64_t buf = ptrace(PTRACE_PEEKDATA, pid, ripAfterSyscall, 0);
        // Patch the first two bytes of the memory immediately after this with
        // jmp -2. Either single stepping will take over before this
        // instruction, leaving the rip where it should be, or it will take
        // over after this instruction, -still- leaving the rip where it should
        // be.
        uint64_t newBuf = (buf & ~0xFFFF) | 0xFEEB;
        // Write the patched memory to the processes address space
        ptrace(PTRACE_POKEDATA, pid, ripAfterSyscall, newBuf);
        // Step and hit it
        ptraceSingleStep();
        // Put things back to the way they started
        ptrace(PTRACE_POKEDATA, pid, ripAfterSyscall, buf);
    } else {
        // Get all the way past repe and repne string instructions in one shot.
        uint64_t newPC, origPC = getPC();
        do {
            ptraceSingleStep();
            newPC = getPC();
        } while (newPC == origPC);
    }
}

TraceChild *
genTraceChild()
{
    return new AMD64TraceChild;
}
