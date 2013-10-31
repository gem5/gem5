/*
 * Copyright (c) 2010-2012 ARM Limited
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
#include "base/compiler.hh"

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
        MISCREG_CPSR_Q,
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
        MISCREG_FPSCR_QC,  // Cumulative saturation flag
        MISCREG_FPSCR_EXC,  // Cumulative FP exception flags
        MISCREG_FPEXC,
        MISCREG_MVFR0,
        MISCREG_MVFR1,
        MISCREG_SCTLR_RST,
        MISCREG_SEV_MAILBOX,

        // CP14 registers
        MISCREG_CP14_START,
        MISCREG_DBGDIDR = MISCREG_CP14_START,
        MISCREG_DBGDSCR_INT,
        MISCREG_DBGDTRRX_INT,
        MISCREG_DBGTRTX_INT,
        MISCREG_DBGWFAR,
        MISCREG_DBGVCR,
        MISCREG_DBGECR,
        MISCREG_DBGDSCCR,
        MISCREG_DBGSMCR,
        MISCREG_DBGDTRRX_EXT,
        MISCREG_DBGDSCR_EXT,
        MISCREG_DBGDTRTX_EXT,
        MISCREG_DBGDRCR,
        MISCREG_DBGBVR,
        MISCREG_DBGBCR,
        MISCREG_DBGBVR_M,
        MISCREG_DBGBCR_M,
        MISCREG_DBGDRAR,
        MISCREG_DBGBXVR_M,
        MISCREG_DBGOSLAR,
        MISCREG_DBGOSSRR,
        MISCREG_DBGOSDLR,
        MISCREG_DBGPRCR,
        MISCREG_DBGPRSR,
        MISCREG_DBGDSAR,
        MISCREG_DBGITCTRL,
        MISCREG_DBGCLAIMSET,
        MISCREG_DBGCLAIMCLR,
        MISCREG_DBGAUTHSTATUS,
        MISCREG_DBGDEVID2,
        MISCREG_DBGDEVID1,
        MISCREG_DBGDEVID,
        MISCREG_TEEHBR,

        // CP15 registers
        MISCREG_CP15_START,
        MISCREG_SCTLR = MISCREG_CP15_START,
        MISCREG_DCCISW,
        MISCREG_DCCIMVAC,
        MISCREG_DCCMVAC,
        MISCREG_CONTEXTIDR,
        MISCREG_TPIDRURW,
        MISCREG_TPIDRURO,
        MISCREG_TPIDRPRW,
        MISCREG_CP15ISB,
        MISCREG_CP15DSB,
        MISCREG_CP15DMB,
        MISCREG_CPACR,
        MISCREG_CLIDR,
        MISCREG_CCSIDR,
        MISCREG_CSSELR,
        MISCREG_ICIALLUIS,
        MISCREG_ICIALLU,
        MISCREG_ICIMVAU,
        MISCREG_BPIMVA,
        MISCREG_BPIALLIS,
        MISCREG_BPIALL,
        MISCREG_MIDR,
        MISCREG_TTBR0,
        MISCREG_TTBR1,
        MISCREG_TLBTR,
        MISCREG_DACR,
        MISCREG_TLBIALLIS,
        MISCREG_TLBIMVAIS,
        MISCREG_TLBIASIDIS,
        MISCREG_TLBIMVAAIS,
        MISCREG_ITLBIALL,
        MISCREG_ITLBIMVA,
        MISCREG_ITLBIASID,
        MISCREG_DTLBIALL,
        MISCREG_DTLBIMVA,
        MISCREG_DTLBIASID,
        MISCREG_TLBIALL,
        MISCREG_TLBIMVA,
        MISCREG_TLBIASID,
        MISCREG_TLBIMVAA,
        MISCREG_DFSR,
        MISCREG_IFSR,
        MISCREG_DFAR,
        MISCREG_IFAR,
        MISCREG_MPIDR,
        MISCREG_PRRR,
        MISCREG_NMRR,
        MISCREG_TTBCR,
        MISCREG_ID_PFR0,
        MISCREG_CTR,
        MISCREG_SCR,
        MISCREG_SDER,
        MISCREG_PAR,
        MISCREG_V2PCWPR,
        MISCREG_V2PCWPW,
        MISCREG_V2PCWUR,
        MISCREG_V2PCWUW,
        MISCREG_V2POWPR,
        MISCREG_V2POWPW,
        MISCREG_V2POWUR,
        MISCREG_V2POWUW,
        MISCREG_ID_MMFR0,
        MISCREG_ID_MMFR2,
        MISCREG_ID_MMFR3,
        MISCREG_ACTLR,
        MISCREG_PMCR,
        MISCREG_PMCCNTR,
        MISCREG_PMCNTENSET,
        MISCREG_PMCNTENCLR,
        MISCREG_PMOVSR,
        MISCREG_PMSWINC,
        MISCREG_PMSELR,
        MISCREG_PMCEID0,
        MISCREG_PMCEID1,
        MISCREG_PMC_OTHER,
        MISCREG_PMXEVCNTR,
        MISCREG_PMUSERENR,
        MISCREG_PMINTENSET,
        MISCREG_PMINTENCLR,
        MISCREG_ID_ISAR0,
        MISCREG_ID_ISAR1,
        MISCREG_ID_ISAR2,
        MISCREG_ID_ISAR3,
        MISCREG_ID_ISAR4,
        MISCREG_ID_ISAR5,
        MISCREG_LOCKFLAG,
        MISCREG_LOCKADDR,
        MISCREG_ID_PFR1,
        MISCREG_L2CTLR,
        MISCREG_CP15_UNIMP_START,
        MISCREG_TCMTR = MISCREG_CP15_UNIMP_START,
        MISCREG_ID_DFR0,
        MISCREG_ID_AFR0,
        MISCREG_ID_MMFR1,
        MISCREG_AIDR,
        MISCREG_ADFSR,
        MISCREG_AIFSR,
        MISCREG_DCIMVAC,
        MISCREG_DCISW,
        MISCREG_MCCSW,
        MISCREG_DCCMVAU,
        MISCREG_NSACR,
        MISCREG_VBAR,
        MISCREG_MVBAR,
        MISCREG_ISR,
        MISCREG_FCEIDR,
        MISCREG_L2LATENCY,
        MISCREG_CRN15,


        MISCREG_CP15_END,

        // Dummy indices
        MISCREG_NOP = MISCREG_CP15_END,
        MISCREG_RAZ,

        NUM_MISCREGS
    };

    MiscRegIndex decodeCP14Reg(unsigned crn, unsigned opc1,
                               unsigned crm, unsigned opc2);

    MiscRegIndex decodeCP15Reg(unsigned crn, unsigned opc1,
                               unsigned crm, unsigned opc2);


    const char * const miscRegName[] = {
        "cpsr", "cpsr_q", "spsr", "spsr_fiq", "spsr_irq", "spsr_svc",
        "spsr_mon", "spsr_und", "spsr_abt",
        "fpsr", "fpsid", "fpscr", "fpscr_qc", "fpscr_exc", "fpexc",
        "mvfr0", "mvfr1",
        "sctlr_rst", "sev_mailbox",
        "DBGDIDR",
        "DBGDSCR_INT",
        "DBGDTRRX_INT",
        "DBGTRTX_INT",
        "DBGWFAR",
        "DBGVCR",
        "DBGECR",
        "DBGDSCCR",
        "DBGSMCR",
        "DBGDTRRX_EXT",
        "DBGDSCR_EXT",
        "DBGDTRTX_EXT",
        "DBGDRCR",
        "DBGBVR",
        "DBGBCR",
        "DBGBVR_M",
        "DBGBCR_M",
        "DBGDRAR",
        "DBGBXVR_M",
        "DBGOSLAR",
        "DBGOSSRR",
        "DBGOSDLR",
        "DBGPRCR",
        "DBGPRSR",
        "DBGDSAR",
        "DBGITCTRL",
        "DBGCLAIMSET",
        "DBGCLAIMCLR",
        "DBGAUTHSTATUS",
        "DBGDEVID2",
        "DBGDEVID1",
        "DBGDEVID",
        "TEEHBR",
        "sctlr", "dccisw", "dccimvac", "dccmvac",
        "contextidr", "tpidrurw", "tpidruro", "tpidrprw",
        "cp15isb", "cp15dsb", "cp15dmb", "cpacr",
        "clidr", "ccsidr", "csselr",
        "icialluis", "iciallu", "icimvau",
        "bpimva", "bpiallis", "bpiall",
        "midr", "ttbr0", "ttbr1", "tlbtr", "dacr",
        "tlbiallis", "tlbimvais", "tlbiasidis", "tlbimvaais",
        "itlbiall", "itlbimva", "itlbiasid",
        "dtlbiall", "dtlbimva", "dtlbiasid",
        "tlbiall", "tlbimva", "tlbiasid", "tlbimvaa",
        "dfsr", "ifsr", "dfar", "ifar", "mpidr",
        "prrr", "nmrr",  "ttbcr", "id_pfr0", "ctr",
        "scr", "sder", "par",
        "v2pcwpr", "v2pcwpw", "v2pcwur", "v2pcwuw",
        "v2powpr", "v2powpw", "v2powur", "v2powuw",
        "id_mmfr0", "id_mmfr2", "id_mmfr3", "actlr", "pmcr", "pmccntr",
        "pmcntenset", "pmcntenclr", "pmovsr",
        "pmswinc", "pmselr", "pmceid0",
        "pmceid1", "pmc_other", "pmxevcntr",
        "pmuserenr", "pmintenset", "pmintenclr",
        "id_isar0", "id_isar1", "id_isar2", "id_isar3", "id_isar4", "id_isar5",
        "lockflag", "lockaddr", "id_pfr1",
        "l2ctlr",
         // Unimplemented below
        "tcmtr",
        "id_dfr0", "id_afr0",
        "id_mmfr1",
        "aidr", "adfsr", "aifsr",
        "dcimvac", "dcisw", "mccsw",
        "dccmvau",
        "nsacr",
        "vbar", "mvbar", "isr", "fceidr", "l2latency",
        "crn15",
        "nop", "raz"
    };

    static_assert(sizeof(miscRegName) / sizeof(*miscRegName) == NUM_MISCREGS,
                  "The miscRegName array and NUM_MISCREGS are inconsistent.");

    BitUnion32(CPSR)
        Bitfield<31,30> nz;
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
    static const uint32_t CondCodesMask   = 0xF00F0000;
    static const uint32_t CpsrMaskQ       = 0x08000000;

    BitUnion32(SCTLR)
        Bitfield<31> ie;  // Instruction endianness
        Bitfield<30> te;  // Thumb Exception Enable
        Bitfield<29> afe; // Access flag enable
        Bitfield<28> tre; // TEX Remap bit
        Bitfield<27> nmfi;// Non-maskable fast interrupts enable
        Bitfield<25> ee;  // Exception Endianness bit
        Bitfield<24> ve;  // Interrupt vectors enable
        Bitfield<23> xp; //  Extended page table enable bit
        Bitfield<22> u;   // Alignment (now unused)
        Bitfield<21> fi;  // Fast interrupts configuration enable
        Bitfield<19> dz;  // Divide by Zero fault enable bit
        Bitfield<18> rao2;// Read as one
        Bitfield<17> br;  // Background region bit
        Bitfield<16> rao3;// Read as one
        Bitfield<14> rr;  // Round robin cache replacement
        Bitfield<13> v;   // Base address for exception vectors
        Bitfield<12> i;   // instruction cache enable
        Bitfield<11> z;   // branch prediction enable bit
        Bitfield<10> sw;  // Enable swp/swpb
        Bitfield<9,8> rs;   // deprecated protection bits
        Bitfield<6,3> rao4;// Read as one
        Bitfield<7>  b;   // Endianness support (unused)
        Bitfield<2>  c;   // Cache enable bit
        Bitfield<1>  a;   // Alignment fault checking
        Bitfield<0>  m;   // MMU enable bit
    EndBitUnion(SCTLR)

    BitUnion32(CPACR)
        Bitfield<1, 0> cp0;
        Bitfield<3, 2> cp1;
        Bitfield<5, 4> cp2;
        Bitfield<7, 6> cp3;
        Bitfield<9, 8> cp4;
        Bitfield<11, 10> cp5;
        Bitfield<13, 12> cp6;
        Bitfield<15, 14> cp7;
        Bitfield<17, 16> cp8;
        Bitfield<19, 18> cp9;
        Bitfield<21, 20> cp10;
        Bitfield<23, 22> cp11;
        Bitfield<25, 24> cp12;
        Bitfield<27, 26> cp13;
        Bitfield<29, 28> rsvd;
        Bitfield<30> d32dis;
        Bitfield<31> asedis;
    EndBitUnion(CPACR)

    BitUnion32(FSR)
        Bitfield<3, 0> fsLow;
        Bitfield<7, 4> domain;
        Bitfield<10> fsHigh;
        Bitfield<11> wnr;
        Bitfield<12> ext;
    EndBitUnion(FSR)

    BitUnion32(FPSCR)
        Bitfield<0> ioc;
        Bitfield<1> dzc;
        Bitfield<2> ofc;
        Bitfield<3> ufc;
        Bitfield<4> ixc;
        Bitfield<7> idc;
        Bitfield<8> ioe;
        Bitfield<9> dze;
        Bitfield<10> ofe;
        Bitfield<11> ufe;
        Bitfield<12> ixe;
        Bitfield<15> ide;
        Bitfield<18, 16> len;
        Bitfield<21, 20> stride;
        Bitfield<23, 22> rMode;
        Bitfield<24> fz;
        Bitfield<25> dn;
        Bitfield<26> ahp;
        Bitfield<27> qc;
        Bitfield<28> v;
        Bitfield<29> c;
        Bitfield<30> z;
        Bitfield<31> n;
    EndBitUnion(FPSCR)

    // This mask selects bits of the FPSCR that actually go in the FpCondCodes
    // integer register to allow renaming.
    static const uint32_t FpCondCodesMask = 0xF0000000;
    // This mask selects the cumulative FP exception flags of the FPSCR.
    static const uint32_t FpscrExcMask = 0x0000009F;
    // This mask selects the cumulative saturation flag of the FPSCR.
    static const uint32_t FpscrQcMask = 0x08000000;

    BitUnion32(FPEXC)
        Bitfield<31> ex;
        Bitfield<30> en;
        Bitfield<29, 0> subArchDefined;
    EndBitUnion(FPEXC)

    BitUnion32(MVFR0)
        Bitfield<3, 0> advSimdRegisters;
        Bitfield<7, 4> singlePrecision;
        Bitfield<11, 8> doublePrecision;
        Bitfield<15, 12> vfpExceptionTrapping;
        Bitfield<19, 16> divide;
        Bitfield<23, 20> squareRoot;
        Bitfield<27, 24> shortVectors;
        Bitfield<31, 28> roundingModes;
    EndBitUnion(MVFR0)

    BitUnion32(MVFR1)
        Bitfield<3, 0> flushToZero;
        Bitfield<7, 4> defaultNaN;
        Bitfield<11, 8> advSimdLoadStore;
        Bitfield<15, 12> advSimdInteger;
        Bitfield<19, 16> advSimdSinglePrecision;
        Bitfield<23, 20> advSimdHalfPrecision;
        Bitfield<27, 24> vfpHalfPrecision;
        Bitfield<31, 28> raz;
    EndBitUnion(MVFR1)

    BitUnion32(PRRR)
       Bitfield<1,0> tr0;
       Bitfield<3,2> tr1;
       Bitfield<5,4> tr2;
       Bitfield<7,6> tr3;
       Bitfield<9,8> tr4;
       Bitfield<11,10> tr5;
       Bitfield<13,12> tr6;
       Bitfield<15,14> tr7;
       Bitfield<16> ds0;
       Bitfield<17> ds1;
       Bitfield<18> ns0;
       Bitfield<19> ns1;
       Bitfield<24> nos0;
       Bitfield<25> nos1;
       Bitfield<26> nos2;
       Bitfield<27> nos3;
       Bitfield<28> nos4;
       Bitfield<29> nos5;
       Bitfield<30> nos6;
       Bitfield<31> nos7;
   EndBitUnion(PRRR)

   BitUnion32(NMRR)
       Bitfield<1,0> ir0;
       Bitfield<3,2> ir1;
       Bitfield<5,4> ir2;
       Bitfield<7,6> ir3;
       Bitfield<9,8> ir4;
       Bitfield<11,10> ir5;
       Bitfield<13,12> ir6;
       Bitfield<15,14> ir7;
       Bitfield<17,16> or0;
       Bitfield<19,18> or1;
       Bitfield<21,20> or2;
       Bitfield<23,22> or3;
       Bitfield<25,24> or4;
       Bitfield<27,26> or5;
       Bitfield<29,28> or6;
       Bitfield<31,30> or7;
   EndBitUnion(NMRR)

   BitUnion32(CONTEXTIDR)
      Bitfield<7,0>  asid;
      Bitfield<31,8> procid;
   EndBitUnion(CONTEXTIDR)

   BitUnion32(L2CTLR)
      Bitfield<2,0>   sataRAMLatency;
      Bitfield<4,3>   reserved_4_3;
      Bitfield<5>     dataRAMSetup;
      Bitfield<8,6>   tagRAMLatency;
      Bitfield<9>     tagRAMSetup;
      Bitfield<11,10> dataRAMSlice;
      Bitfield<12>    tagRAMSlice;
      Bitfield<20,13> reserved_20_13;
      Bitfield<21>    eccandParityEnable;
      Bitfield<22>    reserved_22;
      Bitfield<23>    interptCtrlPresent;
      Bitfield<25,24> numCPUs;
      Bitfield<30,26> reserved_30_26;
      Bitfield<31>    l2rstDISABLE_monitor;
   EndBitUnion(L2CTLR)

   BitUnion32(CTR)
      Bitfield<3,0>   iCacheLineSize;
      Bitfield<13,4>  raz_13_4;
      Bitfield<15,14> l1IndexPolicy;
      Bitfield<19,16> dCacheLineSize;
      Bitfield<23,20> erg;
      Bitfield<27,24> cwg;
      Bitfield<28>    raz_28;
      Bitfield<31,29> format;
   EndBitUnion(CTR)
}

#endif // __ARCH_ARM_MISCREGS_HH__
