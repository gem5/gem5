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

namespace gem5
{

namespace MipsISA
{

// Enumerate names for 'Control' Registers in the CPU
// Reference MIPS32 Arch. for Programmers, Vol. III, Ch.8
// (Register Number-Register Select) Summary of Register
//------------------------------------------------------
// The first set of names classify the CP0 names as Register Banks
// for easy indexing when using the 'RD + SEL' index combination
// in CP0 instructions.
enum MiscRegIndex
{
    MISCREG_INDEX = 0,       //Bank 0: 0 - 3
    MISCREG_MVP_CONTROL,
    MISCREG_MVP_CONF0,
    MISCREG_MVP_CONF1,

    MISCREG_CP0_RANDOM = 8,      //Bank 1: 8 - 15
    MISCREG_VPE_CONTROL,
    MISCREG_VPE_CONF0,
    MISCREG_VPE_CONF1,
    MISCREG_YQMASK,
    MISCREG_VPE_SCHEDULE,
    MISCREG_VPE_SCHEFBACK,
    MISCREG_VPE_OPT,

    MISCREG_ENTRYLO0 = 16,   //Bank 2: 16 - 23
    MISCREG_TC_STATUS,
    MISCREG_TC_BIND,
    MISCREG_TC_RESTART,
    MISCREG_TC_HALT,
    MISCREG_TC_CONTEXT,
    MISCREG_TC_SCHEDULE,
    MISCREG_TC_SCHEFBACK,

    MISCREG_ENTRYLO1 = 24,   // Bank 3: 24

    MISCREG_CONTEXT = 32,    // Bank 4: 32 - 33
    MISCREG_CONTEXT_CONFIG,

    MISCREG_PAGEMASK = 40, //Bank 5: 40 - 41
    MISCREG_PAGEGRAIN = 41,

    MISCREG_WIRED = 48,          //Bank 6:48-55
    MISCREG_SRS_CONF0,
    MISCREG_SRS_CONF1,
    MISCREG_SRS_CONF2,
    MISCREG_SRS_CONF3,
    MISCREG_SRS_CONF4,

    MISCREG_HWRENA = 56,         //Bank 7: 56-63

    MISCREG_BADVADDR = 64,       //Bank 8: 64-71

    MISCREG_COUNT = 72,          //Bank 9: 72-79

    MISCREG_ENTRYHI = 80,        //Bank 10: 80-87

    MISCREG_COMPARE = 88,        //Bank 11: 88-95

    MISCREG_STATUS = 96,         //Bank 12: 96-103
    MISCREG_INTCTL,
    MISCREG_SRSCTL,
    MISCREG_SRSMAP,

    MISCREG_CAUSE = 104,         //Bank 13: 104-111

    MISCREG_EPC = 112,           //Bank 14: 112-119

    MISCREG_PRID = 120,          //Bank 15: 120-127,
    MISCREG_EBASE,

    MISCREG_CONFIG = 128,        //Bank 16: 128-135
    MISCREG_CONFIG1,
    MISCREG_CONFIG2,
    MISCREG_CONFIG3,
    MISCREG_CONFIG4,
    MISCREG_CONFIG5,
    MISCREG_CONFIG6,
    MISCREG_CONFIG7,


    MISCREG_LLADDR = 136,        //Bank 17: 136-143

    MISCREG_WATCHLO0 = 144,      //Bank 18: 144-151
    MISCREG_WATCHLO1,
    MISCREG_WATCHLO2,
    MISCREG_WATCHLO3,
    MISCREG_WATCHLO4,
    MISCREG_WATCHLO5,
    MISCREG_WATCHLO6,
    MISCREG_WATCHLO7,

    MISCREG_WATCHHI0 = 152,     //Bank 19: 152-159
    MISCREG_WATCHHI1,
    MISCREG_WATCHHI2,
    MISCREG_WATCHHI3,
    MISCREG_WATCHHI4,
    MISCREG_WATCHHI5,
    MISCREG_WATCHHI6,
    MISCREG_WATCHHI7,

    MISCREG_XCCONTEXT64 = 160, //Bank 20: 160-167

                       //Bank 21: 168-175

                       //Bank 22: 176-183

    MISCREG_DEBUG = 184,       //Bank 23: 184-191
    MISCREG_TRACE_CONTROL1,
    MISCREG_TRACE_CONTROL2,
    MISCREG_USER_TRACE_DATA,
    MISCREG_TRACE_BPC,

    MISCREG_DEPC = 192,        //Bank 24: 192-199

    MISCREG_PERFCNT0 = 200,    //Bank 25: 200-207
    MISCREG_PERFCNT1,
    MISCREG_PERFCNT2,
    MISCREG_PERFCNT3,
    MISCREG_PERFCNT4,
    MISCREG_PERFCNT5,
    MISCREG_PERFCNT6,
    MISCREG_PERFCNT7,

    MISCREG_ERRCTL = 208,      //Bank 26: 208-215

    MISCREG_CACHEERR0 = 216,   //Bank 27: 216-223
    MISCREG_CACHEERR1,
    MISCREG_CACHEERR2,
    MISCREG_CACHEERR3,

    MISCREG_TAGLO0 = 224,      //Bank 28: 224-231
    MISCREG_DATALO1,
    MISCREG_TAGLO2,
    MISCREG_DATALO3,
    MISCREG_TAGLO4,
    MISCREG_DATALO5,
    MISCREG_TAGLO6,
    MISCREG_DATALO7,

    MISCREG_TAGHI0 = 232,      //Bank 29: 232-239
    MISCREG_DATAHI1,
    MISCREG_TAGHI2,
    MISCREG_DATAHI3,
    MISCREG_TAGHI4,
    MISCREG_DATAHI5,
    MISCREG_TAGHI6,
    MISCREG_DATAHI7,


    MISCREG_ERROR_EPC = 240,    //Bank 30: 240-247

    MISCREG_DESAVE = 248,       //Bank 31: 248-256

    MISCREG_LLFLAG = 257,
    MISCREG_TP_VALUE,

    MISCREG_NUMREGS
};

} // namespace MipsISA
} // namespace gem5

#endif
