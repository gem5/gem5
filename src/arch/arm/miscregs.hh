/*
 * Copyright (c) 2009 The Regents of The University of Michigan
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
#ifndef __ARCH_ARM_MISCREGS_HH__
#define __ARCH_ARM_MISCREGS_HH__

#include "base/bitunion.hh"

namespace ArmISA
{
    enum ConditionCode {
        COND_EQ  =   0,
        COND_NE, //  1
        COND_CS, //  2
        COND_CC, //  3
        COND_MI, //  4
        COND_PL, //  5
        COND_VS, //  6
        COND_VC, //  7
        COND_HI, //  8
        COND_LS, //  9
        COND_GE, // 10
        COND_LT, // 11
        COND_GT, // 12
        COND_LE, // 13
        COND_AL, // 14
        COND_NV  // 15
    };

    enum MiscRegIndex {
        MISCREG_CPSR = 0,
        MISCREG_SPSR,
        MISCREG_SPSR_FIQ,
        MISCREG_SPSR_IRQ,
        MISCREG_SPSR_SVC,
        MISCREG_SPSR_MON,
        MISCREG_SPSR_UND,
        MISCREG_SPSR_ABT,
        MISCREG_FPSR,
        MISCREG_FPSID,
        MISCREG_FPSCR,
        MISCREG_FPEXC,
        MISCREG_SCTLR,
        NUM_MISCREGS
    };

    const char * const miscRegName[NUM_MISCREGS] = {
        "cpsr", "spsr", "spsr_fiq", "spsr_irq", "spsr_svc", "spsr_und", 
        "spsr_abt", "fpsr", "fpsid", "fpscr", "fpexc", "sctlr"
    };

    BitUnion32(CPSR)
        Bitfield<31> n;
        Bitfield<30> z;
        Bitfield<29> c;
        Bitfield<28> v;
        Bitfield<27> q;
        Bitfield<26,25> it1;
        Bitfield<24> j;
        Bitfield<19, 16> ge;
        Bitfield<15,10> it2;
        Bitfield<9> e;
        Bitfield<8> a;
        Bitfield<7> i;
        Bitfield<6> f;
        Bitfield<5> t;
        Bitfield<4, 0> mode;
    EndBitUnion(CPSR)

    // This mask selects bits of the CPSR that actually go in the CondCodes
    // integer register to allow renaming.
    static const uint32_t CondCodesMask = 0xF80F0000;

    BitUnion32(SCTLR)
        Bitfield<30> te;  // Thumb Exception Enable
        Bitfield<29> afe; // Access flag enable
        Bitfield<28> tre; // TEX Remap bit 
        Bitfield<27> nmfi;// Non-maskable fast interrupts enable
        Bitfield<25> ee;  // Exception Endianness bit
        Bitfield<24> ve;  // Interrupt vectors enable
        Bitfield<23> rao1;// Read as one
        Bitfield<22> u;   // Alignment (now unused)
        Bitfield<21> fi;  // Fast interrupts configuration enable
        Bitfield<18> rao2;// Read as one
        Bitfield<17> ha;  // Hardware access flag enable
        Bitfield<16> rao3;// Read as one
        Bitfield<14> rr;  // Round robin cache replacement
        Bitfield<13> v;   // Base address for exception vectors
        Bitfield<12> i;   // instruction cache enable
        Bitfield<11> z;   // branch prediction enable bit
        Bitfield<10> sw;  // Enable swp/swpb
        Bitfield<6,3> rao4;// Read as one
        Bitfield<7>  b;   // Endianness support (unused)  
        Bitfield<2>  c;   // Cache enable bit
        Bitfield<1>  a;   // Alignment fault checking
        Bitfield<0>  m;   // MMU enable bit 
    EndBitUnion(SCTLR)
};

#endif // __ARCH_ARM_MISCREGS_HH__
