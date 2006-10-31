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

namespace AlphaISA
{
    ////////////////////////////////////////////////////////////////////////
    //
    //  Internal Processor Reigsters
    //
    enum md_ipr_names
    {
        IPR_ISR = 0x100,		// interrupt summary register
        IPR_ITB_TAG = 0x101,	// ITLB tag register
        IPR_ITB_PTE = 0x102,	// ITLB page table entry register
        IPR_ITB_ASN = 0x103,	// ITLB address space register
        IPR_ITB_PTE_TEMP = 0x104,	// ITLB page table entry temp register
        IPR_ITB_IA = 0x105,		// ITLB invalidate all register
        IPR_ITB_IAP = 0x106,	// ITLB invalidate all process register
        IPR_ITB_IS = 0x107,		// ITLB invalidate select register
        IPR_SIRR = 0x108,		// software interrupt request register
        IPR_ASTRR = 0x109,		// asynchronous system trap request register
        IPR_ASTER = 0x10a,		// asynchronous system trap enable register
        IPR_EXC_ADDR = 0x10b,	// exception address register
        IPR_EXC_SUM = 0x10c,	// exception summary register
        IPR_EXC_MASK = 0x10d,	// exception mask register
        IPR_PAL_BASE = 0x10e,	// PAL base address register
        IPR_ICM = 0x10f,		// instruction current mode
        IPR_IPLR = 0x110,		// interrupt priority level register
        IPR_INTID = 0x111,		// interrupt ID register
        IPR_IFAULT_VA_FORM = 0x112,	// formatted faulting virtual addr register
        IPR_IVPTBR = 0x113,		// virtual page table base register
        IPR_HWINT_CLR = 0x115,	// H/W interrupt clear register
        IPR_SL_XMIT = 0x116,	// serial line transmit register
        IPR_SL_RCV = 0x117,		// serial line receive register
        IPR_ICSR = 0x118,		// instruction control and status register
        IPR_IC_FLUSH = 0x119,	// instruction cache flush control
        IPR_IC_PERR_STAT = 0x11a,	// inst cache parity error status register
        IPR_PMCTR = 0x11c,		// performance counter register

        // PAL temporary registers...
        // register meanings gleaned from osfpal.s source code
        IPR_PALtemp0 = 0x140,	// local scratch
        IPR_PALtemp1 = 0x141,	// local scratch
        IPR_PALtemp2 = 0x142,	// entUna
        IPR_PALtemp3 = 0x143,	// CPU specific impure area pointer
        IPR_PALtemp4 = 0x144,	// memory management temp
        IPR_PALtemp5 = 0x145,	// memory management temp
        IPR_PALtemp6 = 0x146,	// memory management temp
        IPR_PALtemp7 = 0x147,	// entIF
        IPR_PALtemp8 = 0x148,	// intmask
        IPR_PALtemp9 = 0x149,	// entSys
        IPR_PALtemp10 = 0x14a,	// ??
        IPR_PALtemp11 = 0x14b,	// entInt
        IPR_PALtemp12 = 0x14c,	// entArith
        IPR_PALtemp13 = 0x14d,	// reserved for platform specific PAL
        IPR_PALtemp14 = 0x14e,	// reserved for platform specific PAL
        IPR_PALtemp15 = 0x14f,	// reserved for platform specific PAL
        IPR_PALtemp16 = 0x150,	// scratch / whami<7:0> / mces<4:0>
        IPR_PALtemp17 = 0x151,	// sysval
        IPR_PALtemp18 = 0x152,	// usp
        IPR_PALtemp19 = 0x153,	// ksp
        IPR_PALtemp20 = 0x154,	// PTBR
        IPR_PALtemp21 = 0x155,	// entMM
        IPR_PALtemp22 = 0x156,	// kgp
        IPR_PALtemp23 = 0x157,	// PCBB

        IPR_DTB_ASN = 0x200,	// DTLB address space number register
        IPR_DTB_CM = 0x201,		// DTLB current mode register
        IPR_DTB_TAG = 0x202,	// DTLB tag register
        IPR_DTB_PTE = 0x203,	// DTLB page table entry register
        IPR_DTB_PTE_TEMP = 0x204,	// DTLB page table entry temporary register

        IPR_MM_STAT = 0x205,	// data MMU fault status register
        IPR_VA = 0x206,		// fault virtual address register
        IPR_VA_FORM = 0x207,	// formatted virtual address register
        IPR_MVPTBR = 0x208,		// MTU virtual page table base register
        IPR_DTB_IAP = 0x209,	// DTLB invalidate all process register
        IPR_DTB_IA = 0x20a,		// DTLB invalidate all register
        IPR_DTB_IS = 0x20b,		// DTLB invalidate single register
        IPR_ALT_MODE = 0x20c,	// alternate mode register
        IPR_CC = 0x20d,		// cycle counter register
        IPR_CC_CTL = 0x20e,		// cycle counter control register
        IPR_MCSR = 0x20f,		// MTU control register

        IPR_DC_FLUSH = 0x210,
        IPR_DC_PERR_STAT = 0x212,	// Dcache parity error status register
        IPR_DC_TEST_CTL = 0x213,	// Dcache test tag control register
        IPR_DC_TEST_TAG = 0x214,	// Dcache test tag register
        IPR_DC_TEST_TAG_TEMP = 0x215, // Dcache test tag temporary register
        IPR_DC_MODE = 0x216,	// Dcache mode register
        IPR_MAF_MODE = 0x217,	// miss address file mode register

        MaxInternalProcRegs		// number of IPR registers
    };

    enum MiscRegIpr
    {
        //Write only
        MinWriteOnlyIpr,
        MISCREG_IPR_HWINT_CLR = MinWriteOnlyIpr,
        MISCREG_IPR_SL_XMIT,
        MISCREG_IPR_DC_FLUSH,
        MISCREG_IPR_IC_FLUSH,
        MISCREG_IPR_ALT_MODE,
        MISCREG_IPR_DTB_IA,
        MISCREG_IPR_DTB_IAP,
        MISCREG_IPR_ITB_IA,
        MaxWriteOnlyIpr,
        MISCREG_IPR_ITB_IAP = MaxWriteOnlyIpr,

        //Read only
        MinReadOnlyIpr,
        MISCREG_IPR_INTID = MinReadOnlyIpr,
        MISCREG_IPR_SL_RCV,
        MISCREG_IPR_MM_STAT,
        MISCREG_IPR_ITB_PTE_TEMP,
        MaxReadOnlyIpr,
        MISCREG_IPR_DTB_PTE_TEMP = MaxReadOnlyIpr,

        MISCREG_IPR_ISR,
        MISCREG_IPR_ITB_TAG,
        MISCREG_IPR_ITB_PTE,
        MISCREG_IPR_ITB_ASN,
        MISCREG_IPR_ITB_IS,
        MISCREG_IPR_SIRR,
        MISCREG_IPR_ASTRR,
        MISCREG_IPR_ASTER,
        MISCREG_IPR_EXC_ADDR,
        MISCREG_IPR_EXC_SUM,
        MISCREG_IPR_EXC_MASK,
        MISCREG_IPR_PAL_BASE,
        MISCREG_IPR_ICM,
        MISCREG_IPR_IPLR,
        MISCREG_IPR_IFAULT_VA_FORM,
        MISCREG_IPR_IVPTBR,
        MISCREG_IPR_ICSR,
        MISCREG_IPR_IC_PERR_STAT,
        MISCREG_IPR_PMCTR,

        // PAL temporary registers...
        // register meanings gleaned from osfpal.s source code
        MISCREG_IPR_PALtemp0,
        MISCREG_IPR_PALtemp1,
        MISCREG_IPR_PALtemp2,
        MISCREG_IPR_PALtemp3,
        MISCREG_IPR_PALtemp4,
        MISCREG_IPR_PALtemp5,
        MISCREG_IPR_PALtemp6,
        MISCREG_IPR_PALtemp7,
        MISCREG_IPR_PALtemp8,
        MISCREG_IPR_PALtemp9,
        MISCREG_IPR_PALtemp10,
        MISCREG_IPR_PALtemp11,
        MISCREG_IPR_PALtemp12,
        MISCREG_IPR_PALtemp13,
        MISCREG_IPR_PALtemp14,
        MISCREG_IPR_PALtemp15,
        MISCREG_IPR_PALtemp16,
        MISCREG_IPR_PALtemp17,
        MISCREG_IPR_PALtemp18,
        MISCREG_IPR_PALtemp19,
        MISCREG_IPR_PALtemp20,
        MISCREG_IPR_PALtemp21,
        MISCREG_IPR_PALtemp22,
        MISCREG_IPR_PALtemp23,

        MISCREG_IPR_DTB_ASN,
        MISCREG_IPR_DTB_CM,
        MISCREG_IPR_DTB_TAG,
        MISCREG_IPR_DTB_PTE,

        MISCREG_IPR_VA,
        MISCREG_IPR_VA_FORM,
        MISCREG_IPR_MVPTBR,
        MISCREG_IPR_DTB_IS,
        MISCREG_IPR_CC,
        MISCREG_IPR_CC_CTL,
        MISCREG_IPR_MCSR,

        MISCREG_IPR_DC_PERR_STAT,
        MISCREG_IPR_DC_TEST_CTL,
        MISCREG_IPR_DC_TEST_TAG,
        MISCREG_IPR_DC_TEST_TAG_TEMP,
        MISCREG_IPR_DC_MODE,
        MISCREG_IPR_MAF_MODE,

        NumInternalProcRegs		// number of IPR registers
    };


    extern md_ipr_names MiscRegIndexToIpr[NumInternalProcRegs];
    extern int IprToMiscRegIndex[MaxInternalProcRegs];

    void initializeIprTable();
}

#endif
