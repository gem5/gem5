/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Korey Sewell
 */

#ifndef __ARCH_MIPS_ISA_TRAITS_HH__
#define __ARCH_MIPS_ISA_TRAITS_HH__

#include "arch/mips/types.hh"
#include "sim/host.hh"

namespace LittleEndianGuest {};

#define TARGET_MIPS

class StaticInstPtr;

namespace MipsISA
{
    using namespace LittleEndianGuest;

    StaticInstPtr decodeInst(ExtMachInst);

    // MIPS DOES a delay slot
    #define ISA_HAS_DELAY_SLOT 1

    const Addr PageShift = 13;
    const Addr PageBytes = ULL(1) << PageShift;
    const Addr PageMask = ~(PageBytes - 1);
    const Addr PageOffset = PageBytes - 1;

    // return a no-op instruction... used for instruction fetch faults
    const ExtMachInst NoopMachInst = 0x00000000;

    // Constants Related to the number of registers
    const int NumIntArchRegs = 32;
    const int NumIntSpecialRegs = 9;
    const int NumFloatArchRegs = 32;
    const int NumFloatSpecialRegs = 5;

    // Static instruction parameters
    const int MaxInstSrcRegs = 5;
    const int MaxInstDestRegs = 4;

    // semantically meaningful register indices
    const int ZeroReg = 0;
    const int AssemblerReg = 1;
    const int ReturnValueReg = 2;
    const int ReturnValueReg1 = 2;
    const int ReturnValueReg2 = 3;

    const int ArgumentReg[] = {4, 5, 6, 7};
    const int NumArgumentRegs = sizeof(ArgumentReg) / sizeof(const int);

    const int KernelReg0 = 26;
    const int KernelReg1 = 27;
    const int GlobalPointerReg = 28;
    const int StackPointerReg = 29;
    const int FramePointerReg = 30;
    const int ReturnAddressReg = 31;

    const int SyscallNumReg = ReturnValueReg1;
    const int SyscallPseudoReturnReg = ReturnValueReg2;
    const int SyscallSuccessReg = ArgumentReg[3];

    const int LogVMPageSize = 13;	// 8K bytes
    const int VMPageSize = (1 << LogVMPageSize);

    const int BranchPredAddrShiftAmt = 2; // instructions are 4-byte aligned

    const int MachineBytes = 4;
    const int WordBytes = 4;
    const int HalfwordBytes = 2;
    const int ByteBytes = 1;

    const int ANNOTE_NONE = 0;
    const uint32_t ITOUCH_ANNOTE = 0xffffffff;

    // Enumerate names for 'Control' Registers in the CPU
    // Reference MIPS32 Arch. for Programmers, Vol. III, Ch.8
    // (Register Number-Register Select) Summary of Register
    //------------------------------------------------------
    // The first set of names classify the CP0 names as Register Banks
    // for easy indexing when using the 'RD + SEL' index combination
    // in CP0 instructions.
    enum MiscRegTags {
        Index = 0,       //Bank 0: 0 - 3
        MVPControl,
        MVPConf0,
        MVPConf1,

        Random = 8,      //Bank 1: 8 - 15
        VPEControl,
        VPEConf0,
        VPEConf1,
        YQMask,
        VPESchedule,
        VPEScheFBack,
        VPEOpt,

        EntryLo0 = 16,   //Bank 2: 16 - 23
        TCStatus,
        TCBind,
        TCRestart,
        TCHalt,
        TCContext,
        TCSchedule,
        TCScheFBack,

        EntryLo1 = 24,   // Bank 3: 24

        Context = 32,    // Bank 4: 32 - 33
        ContextConfig,

        //PageMask = 40, //Bank 5: 40 - 41
        PageGrain = 41,

        Wired = 48,          //Bank 6:48-55
        SRSConf0,
        SRSConf1,
        SRSConf2,
        SRSConf3,
        SRSConf4,

        HWRena = 56,         //Bank 7: 56-63

        BadVAddr = 64,       //Bank 8: 64-71

        Count = 72,          //Bank 9: 72-79

        EntryHi = 80,        //Bank 10: 80-87

        Compare = 88,        //Bank 11: 88-95

        Status = 96,         //Bank 12: 96-103
        IntCtl,
        SRSCtl,
        SRSMap,

        Cause = 104,         //Bank 13: 104-111

        EPC = 112,           //Bank 14: 112-119

        PRId = 120,          //Bank 15: 120-127,
        EBase,

        Config = 128,        //Bank 16: 128-135
        Config1,
        Config2,
        Config3,
        Config4,
        Config5,
        Config6,
        Config7,


        LLAddr = 136,        //Bank 17: 136-143

        WatchLo0 = 144,      //Bank 18: 144-151
        WatchLo1,
        WatchLo2,
        WatchLo3,
        WatchLo4,
        WatchLo5,
        WatchLo6,
        WatchLo7,

        WatchHi0 = 152,     //Bank 19: 152-159
        WatchHi1,
        WatchHi2,
        WatchHi3,
        WatchHi4,
        WatchHi5,
        WatchHi6,
        WatchHi7,

        XCContext64 = 160, //Bank 20: 160-167

                           //Bank 21: 168-175

                           //Bank 22: 176-183

        Debug = 184,       //Bank 23: 184-191
        TraceControl1,
        TraceControl2,
        UserTraceData,
        TraceBPC,

        DEPC = 192,        //Bank 24: 192-199

        PerfCnt0 = 200,    //Bank 25: 200-207
        PerfCnt1,
        PerfCnt2,
        PerfCnt3,
        PerfCnt4,
        PerfCnt5,
        PerfCnt6,
        PerfCnt7,

        ErrCtl = 208,      //Bank 26: 208-215

        CacheErr0 = 216,   //Bank 27: 216-223
        CacheErr1,
        CacheErr2,
        CacheErr3,

        TagLo0 = 224,      //Bank 28: 224-231
        DataLo1,
        TagLo2,
        DataLo3,
        TagLo4,
        DataLo5,
        TagLo6,
        DataLo7,

        TagHi0 = 232,      //Bank 29: 232-239
        DataHi1,
        TagHi2,
        DataHi3,
        TagHi4,
        DataHi5,
        TagHi6,
        DataHi7,


        ErrorEPC = 240,    //Bank 30: 240-247

        DESAVE = 248,       //Bank 31: 248-256

        LLFlag = 257,

        NumControlRegs
    };

    const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs;        //HI & LO Regs
    const int NumFloatRegs = NumFloatArchRegs + NumFloatSpecialRegs;//
    const int NumMiscRegs = NumControlRegs;

    const int TotalNumRegs = NumIntRegs + NumFloatRegs + NumMiscRegs;

    const int TotalDataRegs = NumIntRegs + NumFloatRegs;

    // These help enumerate all the registers for dependence tracking.
    const int FP_Base_DepTag = NumIntRegs;
    const int Ctrl_Base_DepTag = FP_Base_DepTag + NumFloatRegs;
};

using namespace MipsISA;

#endif // __ARCH_MIPS_ISA_TRAITS_HH__
