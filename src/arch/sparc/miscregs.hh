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
 */

#ifndef __ARCH_SPARC_MISCREGS_HH__
#define __ARCH_SPARC_MISCREGS_HH__

#include "base/bitunion.hh"
#include "base/types.hh"

namespace SparcISA
{
enum MiscRegIndex
{
    /** Ancillary State Registers */
//    MISCREG_Y,
//    MISCREG_CCR,
    MISCREG_ASI,
    MISCREG_TICK,
    MISCREG_FPRS,
    MISCREG_PCR,
    MISCREG_PIC,
    MISCREG_GSR,
    MISCREG_SOFTINT_SET,
    MISCREG_SOFTINT_CLR,
    MISCREG_SOFTINT, /* 10 */
    MISCREG_TICK_CMPR,
    MISCREG_STICK,
    MISCREG_STICK_CMPR,

    /** Privilged Registers */
    MISCREG_TPC,
    MISCREG_TNPC,
    MISCREG_TSTATE,
    MISCREG_TT,
    MISCREG_PRIVTICK,
    MISCREG_TBA,
    MISCREG_PSTATE, /* 20 */
    MISCREG_TL,
    MISCREG_PIL,
    MISCREG_CWP,
//    MISCREG_CANSAVE,
//    MISCREG_CANRESTORE,
//    MISCREG_CLEANWIN,
//    MISCREG_OTHERWIN,
//    MISCREG_WSTATE,
    MISCREG_GL,

    /** Hyper privileged registers */
    MISCREG_HPSTATE, /* 30 */
    MISCREG_HTSTATE,
    MISCREG_HINTP,
    MISCREG_HTBA,
    MISCREG_HVER,
    MISCREG_STRAND_STS_REG,
    MISCREG_HSTICK_CMPR,

    /** Floating Point Status Register */
    MISCREG_FSR,

    /** MMU Internal Registers */
    MISCREG_MMU_P_CONTEXT,
    MISCREG_MMU_S_CONTEXT, /* 40 */
    MISCREG_MMU_PART_ID,
    MISCREG_MMU_LSU_CTRL,

    /** Scratchpad regiscers **/
    MISCREG_SCRATCHPAD_R0, /* 60 */
    MISCREG_SCRATCHPAD_R1,
    MISCREG_SCRATCHPAD_R2,
    MISCREG_SCRATCHPAD_R3,
    MISCREG_SCRATCHPAD_R4,
    MISCREG_SCRATCHPAD_R5,
    MISCREG_SCRATCHPAD_R6,
    MISCREG_SCRATCHPAD_R7,

    /* CPU Queue Registers */
    MISCREG_QUEUE_CPU_MONDO_HEAD,
    MISCREG_QUEUE_CPU_MONDO_TAIL,
    MISCREG_QUEUE_DEV_MONDO_HEAD, /* 70 */
    MISCREG_QUEUE_DEV_MONDO_TAIL,
    MISCREG_QUEUE_RES_ERROR_HEAD,
    MISCREG_QUEUE_RES_ERROR_TAIL,
    MISCREG_QUEUE_NRES_ERROR_HEAD,
    MISCREG_QUEUE_NRES_ERROR_TAIL,

    /* All the data for the TLB packed up in one register. */
    MISCREG_TLB_DATA,
    MISCREG_NUMMISCREGS
};

BitUnion64(HPSTATE)
    Bitfield<0> tlz;
    Bitfield<2> hpriv;
    Bitfield<5> red;
    Bitfield<10> ibe;
    Bitfield<11> id;  // this impl. dependent (id) field m
EndBitUnion(HPSTATE)

BitUnion16(PSTATE)
    Bitfield<1> ie;
    Bitfield<2> priv;
    Bitfield<3> am;
    Bitfield<4> pef;
    Bitfield<7, 6> mm;
    Bitfield<8> tle;
    Bitfield<9> cle;
    Bitfield<10> pid0;
    Bitfield<11> pid1;
EndBitUnion(PSTATE)

BitUnion8(CCR)
    SubBitUnion(xcc, 7, 4)
        Bitfield<7> n;
        Bitfield<6> z;
        Bitfield<5> v;
        Bitfield<4> c;
    EndSubBitUnion(xcc)
    SubBitUnion(icc, 3, 0)
        Bitfield<3> n;
        Bitfield<2> z;
        Bitfield<1> v;
        Bitfield<0> c;
    EndSubBitUnion(icc)
EndBitUnion(CCR)

struct STS
{
    const static int st_idle     = 0x00;
    const static int st_wait     = 0x01;
    const static int st_halt     = 0x02;
    const static int st_run      = 0x05;
    const static int st_spec_run = 0x07;
    const static int st_spec_rdy = 0x13;
    const static int st_ready    = 0x19;
    const static int active      = 0x01;
    const static int speculative = 0x04;
    const static int shft_id     = 8;
    const static int shft_fsm0   = 31;
    const static int shft_fsm1   = 26;
    const static int shft_fsm2   = 21;
    const static int shft_fsm3   = 16;
};


const int NumMiscRegs = MISCREG_NUMMISCREGS;

}

#endif
