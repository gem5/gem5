/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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

#ifndef __ARCH_MIPS_REGS_MISC_HH__
#define __ARCH_MIPS_REGS_MISC_HH__

#include "cpu/reg_class.hh"
#include "debug/MiscRegs.hh"

namespace gem5
{
namespace MipsISA
{
namespace misc_reg
{

// Enumerate names for 'Control' Registers in the CPU
// Reference MIPS32 Arch. for Programmers, Vol. III, Ch.8
// (Register Number-Register Select) Summary of Register
//------------------------------------------------------
// The first set of names classify the CP0 names as Register Banks
// for easy indexing when using the 'RD + SEL' index combination
// in CP0 instructions.
enum : RegIndex
{
    Index = 0,       //Bank 0: 0 - 3
    MvpControl,
    MvpConf0,
    MvpConf1,

    Cp0Random = 8,      //Bank 1: 8 - 15
    VpeControl,
    VpeConf0,
    VpeConf1,
    Yqmask,
    VpeSchedule,
    VpeSchefback,
    VpeOpt,

    Entrylo0 = 16,   //Bank 2: 16 - 23
    TcStatus,
    TcBind,
    TcRestart,
    TcHalt,
    TcContext,
    TcSchedule,
    TcSchefback,

    Entrylo1 = 24,   // Bank 3: 24

    Context = 32,    // Bank 4: 32 - 33
    ContextConfig,

    Pagemask = 40, //Bank 5: 40 - 41
    Pagegrain = 41,

    Wired = 48,          //Bank 6:48-55
    SrsConf0,
    SrsConf1,
    SrsConf2,
    SrsConf3,
    SrsConf4,

    Hwrena = 56,         //Bank 7: 56-63

    Badvaddr = 64,       //Bank 8: 64-71

    Count = 72,          //Bank 9: 72-79

    Entryhi = 80,        //Bank 10: 80-87

    Compare = 88,        //Bank 11: 88-95

    Status = 96,         //Bank 12: 96-103
    Intctl,
    Srsctl,
    Srsmap,

    Cause = 104,         //Bank 13: 104-111

    Epc = 112,           //Bank 14: 112-119

    Prid = 120,          //Bank 15: 120-127,
    Ebase,

    Config = 128,        //Bank 16: 128-135
    Config1,
    Config2,
    Config3,
    Config4,
    Config5,
    Config6,
    Config7,


    Lladdr = 136,        //Bank 17: 136-143

    Watchlo0 = 144,      //Bank 18: 144-151
    Watchlo1,
    Watchlo2,
    Watchlo3,
    Watchlo4,
    Watchlo5,
    Watchlo6,
    Watchlo7,

    Watchhi0 = 152,     //Bank 19: 152-159
    Watchhi1,
    Watchhi2,
    Watchhi3,
    Watchhi4,
    Watchhi5,
    Watchhi6,
    Watchhi7,

    Xccontext64 = 160, //Bank 20: 160-167

                       //Bank 21: 168-175

                       //Bank 22: 176-183

    Debug = 184,       //Bank 23: 184-191
    TraceControl1,
    TraceControl2,
    UserTraceData,
    TraceBpc,

    Depc = 192,        //Bank 24: 192-199

    Perfcnt0 = 200,    //Bank 25: 200-207
    Perfcnt1,
    Perfcnt2,
    Perfcnt3,
    Perfcnt4,
    Perfcnt5,
    Perfcnt6,
    Perfcnt7,

    Errctl = 208,      //Bank 26: 208-215

    Cacheerr0 = 216,   //Bank 27: 216-223
    Cacheerr1,
    Cacheerr2,
    Cacheerr3,

    Taglo0 = 224,      //Bank 28: 224-231
    Datalo1,
    Taglo2,
    Datalo3,
    Taglo4,
    Datalo5,
    Taglo6,
    Datalo7,

    Taghi0 = 232,      //Bank 29: 232-239
    Datahi1,
    Taghi2,
    Datahi3,
    Taghi4,
    Datahi5,
    Taghi6,
    Datahi7,


    ErrorEpc = 240,    //Bank 30: 240-247

    Desave = 248,       //Bank 31: 248-256

    Llflag = 257,
    TpValue,

    NumRegs
};

} // namespace misc_reg

inline constexpr RegClass miscRegClass(MiscRegClass, MiscRegClassName,
        misc_reg::NumRegs, debug::MiscRegs);

} // namespace MipsISA
} // namespace gem5

#endif
