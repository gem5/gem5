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
 * Authors: Steve Reinhardt
 *          Gabe Black
 */

#ifndef __ARCH_ALPHA_IPR_HH__
#define __ARCH_ALPHA_IPR_HH__

namespace AlphaISA {

////////////////////////////////////////////////////////////////////////
//
//  Internal Processor Reigsters
//
enum md_ipr_names {
    RAW_IPR_ISR = 0x100,            // interrupt summary
    RAW_IPR_ITB_TAG = 0x101,        // ITLB tag
    RAW_IPR_ITB_PTE = 0x102,        // ITLB page table entry
    RAW_IPR_ITB_ASN = 0x103,        // ITLB address space
    RAW_IPR_ITB_PTE_TEMP = 0x104,   // ITLB page table entry temp
    RAW_IPR_ITB_IA = 0x105,         // ITLB invalidate all
    RAW_IPR_ITB_IAP = 0x106,        // ITLB invalidate all process
    RAW_IPR_ITB_IS = 0x107,         // ITLB invalidate select
    RAW_IPR_SIRR = 0x108,           // software interrupt request
    RAW_IPR_ASTRR = 0x109,          // asynchronous system trap request
    RAW_IPR_ASTER = 0x10a,          // asynchronous system trap enable
    RAW_IPR_EXC_ADDR = 0x10b,       // exception address
    RAW_IPR_EXC_SUM = 0x10c,        // exception summary
    RAW_IPR_EXC_MASK = 0x10d,       // exception mask
    RAW_IPR_PAL_BASE = 0x10e,       // PAL base address
    RAW_IPR_ICM = 0x10f,            // instruction current mode
    RAW_IPR_IPLR = 0x110,           // interrupt priority level
    RAW_IPR_INTID = 0x111,          // interrupt ID
    RAW_IPR_IFAULT_VA_FORM = 0x112, // formatted faulting virtual addr
    RAW_IPR_IVPTBR = 0x113,         // virtual page table base
    RAW_IPR_HWINT_CLR = 0x115,      // H/W interrupt clear
    RAW_IPR_SL_XMIT = 0x116,        // serial line transmit
    RAW_IPR_SL_RCV = 0x117,         // serial line receive
    RAW_IPR_ICSR = 0x118,           // instruction control and status
    RAW_IPR_IC_FLUSH = 0x119,       // instruction cache flush control
    RAW_IPR_IC_PERR_STAT = 0x11a,   // inst cache parity error status
    RAW_IPR_PMCTR = 0x11c,          // performance counter

    // PAL temporary registers...
    // register meanings gleaned from osfpal.s source code
    RAW_IPR_PALtemp0 = 0x140,       // local scratch
    RAW_IPR_PALtemp1 = 0x141,       // local scratch
    RAW_IPR_PALtemp2 = 0x142,       // entUna
    RAW_IPR_PALtemp3 = 0x143,       // CPU specific impure area pointer
    RAW_IPR_PALtemp4 = 0x144,       // memory management temp
    RAW_IPR_PALtemp5 = 0x145,       // memory management temp
    RAW_IPR_PALtemp6 = 0x146,       // memory management temp
    RAW_IPR_PALtemp7 = 0x147,       // entIF
    RAW_IPR_PALtemp8 = 0x148,       // intmask
    RAW_IPR_PALtemp9 = 0x149,       // entSys
    RAW_IPR_PALtemp10 = 0x14a,      // ??
    RAW_IPR_PALtemp11 = 0x14b,      // entInt
    RAW_IPR_PALtemp12 = 0x14c,      // entArith
    RAW_IPR_PALtemp13 = 0x14d,      // reserved for platform specific PAL
    RAW_IPR_PALtemp14 = 0x14e,      // reserved for platform specific PAL
    RAW_IPR_PALtemp15 = 0x14f,      // reserved for platform specific PAL
    RAW_IPR_PALtemp16 = 0x150,      // scratch / whami<7:0> / mces<4:0>
    RAW_IPR_PALtemp17 = 0x151,      // sysval
    RAW_IPR_PALtemp18 = 0x152,      // usp
    RAW_IPR_PALtemp19 = 0x153,      // ksp
    RAW_IPR_PALtemp20 = 0x154,      // PTBR
    RAW_IPR_PALtemp21 = 0x155,      // entMM
    RAW_IPR_PALtemp22 = 0x156,      // kgp
    RAW_IPR_PALtemp23 = 0x157,      // PCBB

    RAW_IPR_DTB_ASN = 0x200,        // DTLB address space number
    RAW_IPR_DTB_CM = 0x201,         // DTLB current mode
    RAW_IPR_DTB_TAG = 0x202,        // DTLB tag
    RAW_IPR_DTB_PTE = 0x203,        // DTLB page table entry
    RAW_IPR_DTB_PTE_TEMP = 0x204,   // DTLB page table entry temporary

    RAW_IPR_MM_STAT = 0x205,        // data MMU fault status
    RAW_IPR_VA = 0x206,             // fault virtual address
    RAW_IPR_VA_FORM = 0x207,        // formatted virtual address
    RAW_IPR_MVPTBR = 0x208,         // MTU virtual page table base
    RAW_IPR_DTB_IAP = 0x209,        // DTLB invalidate all process
    RAW_IPR_DTB_IA = 0x20a,         // DTLB invalidate all
    RAW_IPR_DTB_IS = 0x20b,         // DTLB invalidate single
    RAW_IPR_ALT_MODE = 0x20c,       // alternate mode
    RAW_IPR_CC = 0x20d,             // cycle counter
    RAW_IPR_CC_CTL = 0x20e,         // cycle counter control
    RAW_IPR_MCSR = 0x20f,           // MTU control

    RAW_IPR_DC_FLUSH = 0x210,
    RAW_IPR_DC_PERR_STAT = 0x212,   // Dcache parity error status
    RAW_IPR_DC_TEST_CTL = 0x213,    // Dcache test tag control
    RAW_IPR_DC_TEST_TAG = 0x214,    // Dcache test tag
    RAW_IPR_DC_TEST_TAG_TEMP = 0x215, // Dcache test tag temporary
    RAW_IPR_DC_MODE = 0x216,        // Dcache mode
    RAW_IPR_MAF_MODE = 0x217,       // miss address file mode

    MaxInternalProcRegs             // number of IPRs
};

enum MiscRegIpr
{
    //Write only
    MinWriteOnlyIpr,
    IPR_HWINT_CLR = MinWriteOnlyIpr,
    IPR_SL_XMIT,
    IPR_DC_FLUSH,
    IPR_IC_FLUSH,
    IPR_ALT_MODE,
    IPR_DTB_IA,
    IPR_DTB_IAP,
    IPR_ITB_IA,
    MaxWriteOnlyIpr,
    IPR_ITB_IAP = MaxWriteOnlyIpr,

    //Read only
    MinReadOnlyIpr,
    IPR_INTID = MinReadOnlyIpr,
    IPR_SL_RCV,
    IPR_MM_STAT,
    IPR_ITB_PTE_TEMP,
    MaxReadOnlyIpr,
    IPR_DTB_PTE_TEMP = MaxReadOnlyIpr,

    IPR_ISR,
    IPR_ITB_TAG,
    IPR_ITB_PTE,
    IPR_ITB_ASN,
    IPR_ITB_IS,
    IPR_SIRR,
    IPR_ASTRR,
    IPR_ASTER,
    IPR_EXC_ADDR,
    IPR_EXC_SUM,
    IPR_EXC_MASK,
    IPR_PAL_BASE,
    IPR_ICM,
    IPR_IPLR,
    IPR_IFAULT_VA_FORM,
    IPR_IVPTBR,
    IPR_ICSR,
    IPR_IC_PERR_STAT,
    IPR_PMCTR,

    // PAL temporary registers...
    // register meanings gleaned from osfpal.s source code
    IPR_PALtemp0,
    IPR_PALtemp1,
    IPR_PALtemp2,
    IPR_PALtemp3,
    IPR_PALtemp4,
    IPR_PALtemp5,
    IPR_PALtemp6,
    IPR_PALtemp7,
    IPR_PALtemp8,
    IPR_PALtemp9,
    IPR_PALtemp10,
    IPR_PALtemp11,
    IPR_PALtemp12,
    IPR_PALtemp13,
    IPR_PALtemp14,
    IPR_PALtemp15,
    IPR_PALtemp16,
    IPR_PALtemp17,
    IPR_PALtemp18,
    IPR_PALtemp19,
    IPR_PALtemp20,
    IPR_PALtemp21,
    IPR_PALtemp22,
    IPR_PALtemp23,

    IPR_DTB_ASN,
    IPR_DTB_CM,
    IPR_DTB_TAG,
    IPR_DTB_PTE,

    IPR_VA,
    IPR_VA_FORM,
    IPR_MVPTBR,
    IPR_DTB_IS,
    IPR_CC,
    IPR_CC_CTL,
    IPR_MCSR,

    IPR_DC_PERR_STAT,
    IPR_DC_TEST_CTL,
    IPR_DC_TEST_TAG,
    IPR_DC_TEST_TAG_TEMP,
    IPR_DC_MODE,
    IPR_MAF_MODE,

    NumInternalProcRegs             // number of IPR registers
};

inline bool
IprIsWritable(int index)
{
    return index < MinReadOnlyIpr || index > MaxReadOnlyIpr;
}

inline bool
IprIsReadable(int index)
{
    return index < MinWriteOnlyIpr || index > MaxWriteOnlyIpr;
}

extern md_ipr_names MiscRegIndexToIpr[NumInternalProcRegs];
extern int IprToMiscRegIndex[MaxInternalProcRegs];

void initializeIprTable();

} // namespace AlphaISA

#endif // __ARCH_ALPHA_IPR_HH__
