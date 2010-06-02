/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
        COND_UC  // 15
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

        // CP15 registers
        MISCREG_CP15_START,
        MISCREG_SCTLR = MISCREG_CP15_START,
        MISCREG_DCCISW,
        MISCREG_CP15_UNIMP_START,
        MISCREG_CTR = MISCREG_CP15_UNIMP_START,
        MISCREG_TCMTR,
        MISCREG_MPUIR,
        MISCREG_MPIDR,
        MISCREG_MIDR,
        MISCREG_ID_PFR0,
        MISCREG_ID_PFR1,
        MISCREG_ID_DFR0,
        MISCREG_ID_AFR0,
        MISCREG_ID_MMFR0,
        MISCREG_ID_MMFR1,
        MISCREG_ID_MMFR2,
        MISCREG_ID_MMFR3,
        MISCREG_ID_ISAR0,
        MISCREG_ID_ISAR1,
        MISCREG_ID_ISAR2,
        MISCREG_ID_ISAR3,
        MISCREG_ID_ISAR4,
        MISCREG_ID_ISAR5,
        MISCREG_CCSIDR,
        MISCREG_CLIDR,
        MISCREG_AIDR,
        MISCREG_CSSELR,
        MISCREG_ACTLR,
        MISCREG_CPACR,
        MISCREG_DFSR,
        MISCREG_IFSR,
        MISCREG_ADFSR,
        MISCREG_AIFSR,
        MISCREG_DFAR,
        MISCREG_IFAR,
        MISCREG_DRBAR,
        MISCREG_IRBAR,
        MISCREG_DRSR,
        MISCREG_IRSR,
        MISCREG_DRACR,
        MISCREG_IRACR,
        MISCREG_RGNR,
        MISCREG_ICIALLUIS,
        MISCREG_BPIALLIS,
        MISCREG_ICIALLU,
        MISCREG_ICIMVAU,
        MISCREG_CP15ISB,
        MISCREG_BPIALL,
        MISCREG_BPIMVA,
        MISCREG_DCIMVAC,
        MISCREG_DCISW,
        MISCREG_DCCMVAC,
        MISCREG_MCCSW,
        MISCREG_CP15DSB,
        MISCREG_CP15DMB,
        MISCREG_DCCMVAU,
        MISCREG_DCCIMVAC,
        MISCREG_CONTEXTIDR,
        MISCREG_TPIDRURW,
        MISCREG_TPIDRURO,
        MISCREG_TPIDRPRW,

        MISCREG_CP15_END,

        // Dummy indices
        MISCREG_NOP = MISCREG_CP15_END,
        MISCREG_RAZ,

        NUM_MISCREGS
    };

    MiscRegIndex decodeCP15Reg(unsigned crn, unsigned opc1,
                               unsigned crm, unsigned opc2);

    const char * const miscRegName[NUM_MISCREGS] = {
        "cpsr", "spsr", "spsr_fiq", "spsr_irq", "spsr_svc",
        "spsr_mon", "spsr_und", "spsr_abt",
        "fpsr", "fpsid", "fpscr", "fpexc",
        "sctlr", "dccisw", "ctr", "tcmtr", "mpuir", "mpidr", "midr",
        "id_pfr0", "id_pfr1", "id_dfr0", "id_afr0",
        "id_mmfr0", "id_mmfr1", "id_mmfr2", "id_mmfr3",
        "id_isar0", "id_isar1", "id_isar2", "id_isar3", "id_isar4", "id_isar5",
        "ccsidr", "clidr", "aidr", "csselr", "actlr", "cpacr",
        "dfsr", "ifsr", "adfsr", "aifsr", "dfar", "ifar",
        "drbar", "irbar", "drsr", "irsr", "dracr", "iracr",
        "rgnr", "icialluis", "bpiallis", "iciallu", "icimvau",
        "cp15isb", "bpiall", "bpimva", "dcimvac", "dcisw", "dccmvac", "mccsw",
        "cp15dsb", "cp15dmb", "dccmvau", "dccimvac",
        "contextidr", "tpidrurw", "tpidruro", "tpidrprw",
        "nop", "raz"
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

    // These otherwise unused bits of the PC are used to select a mode
    // like the J and T bits of the CPSR.
    static const Addr PcJBitShift = 33;
    static const Addr PcTBitShift = 34;
    static const Addr PcModeMask = (ULL(1) << PcJBitShift) |
                                   (ULL(1) << PcTBitShift);

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
