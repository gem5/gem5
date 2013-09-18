/*
 * Copyright (c) 2011 Google
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

#include "arch/x86/regs/msr.hh"

namespace X86ISA
{

typedef MsrMap::value_type MsrVal;

const MsrMap::value_type msrMapData[] = {
    MsrVal(0x10, MISCREG_TSC),
    MsrVal(0x1B, MISCREG_APIC_BASE),
    MsrVal(0xFE, MISCREG_MTRRCAP),
    MsrVal(0x174, MISCREG_SYSENTER_CS),
    MsrVal(0x175, MISCREG_SYSENTER_ESP),
    MsrVal(0x176, MISCREG_SYSENTER_EIP),
    MsrVal(0x179, MISCREG_MCG_CAP),
    MsrVal(0x17A, MISCREG_MCG_STATUS),
    MsrVal(0x17B, MISCREG_MCG_CTL),
    MsrVal(0x1D9, MISCREG_DEBUG_CTL_MSR),
    MsrVal(0x1DB, MISCREG_LAST_BRANCH_FROM_IP),
    MsrVal(0x1DC, MISCREG_LAST_BRANCH_TO_IP),
    MsrVal(0x1DD, MISCREG_LAST_EXCEPTION_FROM_IP),
    MsrVal(0x1DE, MISCREG_LAST_EXCEPTION_TO_IP),
    MsrVal(0x200, MISCREG_MTRR_PHYS_BASE_0),
    MsrVal(0x201, MISCREG_MTRR_PHYS_MASK_0),
    MsrVal(0x202, MISCREG_MTRR_PHYS_BASE_1),
    MsrVal(0x203, MISCREG_MTRR_PHYS_MASK_1),
    MsrVal(0x204, MISCREG_MTRR_PHYS_BASE_2),
    MsrVal(0x205, MISCREG_MTRR_PHYS_MASK_2),
    MsrVal(0x206, MISCREG_MTRR_PHYS_BASE_3),
    MsrVal(0x207, MISCREG_MTRR_PHYS_MASK_3),
    MsrVal(0x208, MISCREG_MTRR_PHYS_BASE_4),
    MsrVal(0x209, MISCREG_MTRR_PHYS_MASK_4),
    MsrVal(0x20A, MISCREG_MTRR_PHYS_BASE_5),
    MsrVal(0x20B, MISCREG_MTRR_PHYS_MASK_5),
    MsrVal(0x20C, MISCREG_MTRR_PHYS_BASE_6),
    MsrVal(0x20D, MISCREG_MTRR_PHYS_MASK_6),
    MsrVal(0x20E, MISCREG_MTRR_PHYS_BASE_7),
    MsrVal(0x20F, MISCREG_MTRR_PHYS_MASK_7),
    MsrVal(0x250, MISCREG_MTRR_FIX_64K_00000),
    MsrVal(0x258, MISCREG_MTRR_FIX_16K_80000),
    MsrVal(0x259, MISCREG_MTRR_FIX_16K_A0000),
    MsrVal(0x268, MISCREG_MTRR_FIX_4K_C0000),
    MsrVal(0x269, MISCREG_MTRR_FIX_4K_C8000),
    MsrVal(0x26A, MISCREG_MTRR_FIX_4K_D0000),
    MsrVal(0x26B, MISCREG_MTRR_FIX_4K_D8000),
    MsrVal(0x26C, MISCREG_MTRR_FIX_4K_E0000),
    MsrVal(0x26D, MISCREG_MTRR_FIX_4K_E8000),
    MsrVal(0x26E, MISCREG_MTRR_FIX_4K_F0000),
    MsrVal(0x26F, MISCREG_MTRR_FIX_4K_F8000),
    MsrVal(0x277, MISCREG_PAT),
    MsrVal(0x2FF, MISCREG_DEF_TYPE),
    MsrVal(0x400, MISCREG_MC0_CTL),
    MsrVal(0x404, MISCREG_MC1_CTL),
    MsrVal(0x408, MISCREG_MC2_CTL),
    MsrVal(0x40C, MISCREG_MC3_CTL),
    MsrVal(0x410, MISCREG_MC4_CTL),
    MsrVal(0x414, MISCREG_MC5_CTL),
    MsrVal(0x418, MISCREG_MC6_CTL),
    MsrVal(0x41C, MISCREG_MC7_CTL),
    MsrVal(0x401, MISCREG_MC0_STATUS),
    MsrVal(0x405, MISCREG_MC1_STATUS),
    MsrVal(0x409, MISCREG_MC2_STATUS),
    MsrVal(0x40D, MISCREG_MC3_STATUS),
    MsrVal(0x411, MISCREG_MC4_STATUS),
    MsrVal(0x415, MISCREG_MC5_STATUS),
    MsrVal(0x419, MISCREG_MC6_STATUS),
    MsrVal(0x41D, MISCREG_MC7_STATUS),
    MsrVal(0x402, MISCREG_MC0_ADDR),
    MsrVal(0x406, MISCREG_MC1_ADDR),
    MsrVal(0x40A, MISCREG_MC2_ADDR),
    MsrVal(0x40E, MISCREG_MC3_ADDR),
    MsrVal(0x412, MISCREG_MC4_ADDR),
    MsrVal(0x416, MISCREG_MC5_ADDR),
    MsrVal(0x41A, MISCREG_MC6_ADDR),
    MsrVal(0x41E, MISCREG_MC7_ADDR),
    MsrVal(0x403, MISCREG_MC0_MISC),
    MsrVal(0x407, MISCREG_MC1_MISC),
    MsrVal(0x40B, MISCREG_MC2_MISC),
    MsrVal(0x40F, MISCREG_MC3_MISC),
    MsrVal(0x413, MISCREG_MC4_MISC),
    MsrVal(0x417, MISCREG_MC5_MISC),
    MsrVal(0x41B, MISCREG_MC6_MISC),
    MsrVal(0x41F, MISCREG_MC7_MISC),
    MsrVal(0xC0000080, MISCREG_EFER),
    MsrVal(0xC0000081, MISCREG_STAR),
    MsrVal(0xC0000082, MISCREG_LSTAR),
    MsrVal(0xC0000083, MISCREG_CSTAR),
    MsrVal(0xC0000084, MISCREG_SF_MASK),
    MsrVal(0xC0000100, MISCREG_FS_BASE),
    MsrVal(0xC0000101, MISCREG_GS_BASE),
    MsrVal(0xC0000102, MISCREG_KERNEL_GS_BASE),
    MsrVal(0xC0000103, MISCREG_TSC_AUX),
    MsrVal(0xC0010000, MISCREG_PERF_EVT_SEL0),
    MsrVal(0xC0010001, MISCREG_PERF_EVT_SEL1),
    MsrVal(0xC0010002, MISCREG_PERF_EVT_SEL2),
    MsrVal(0xC0010003, MISCREG_PERF_EVT_SEL3),
    MsrVal(0xC0010004, MISCREG_PERF_EVT_CTR0),
    MsrVal(0xC0010005, MISCREG_PERF_EVT_CTR1),
    MsrVal(0xC0010006, MISCREG_PERF_EVT_CTR2),
    MsrVal(0xC0010007, MISCREG_PERF_EVT_CTR3),
    MsrVal(0xC0010010, MISCREG_SYSCFG),
    MsrVal(0xC0010016, MISCREG_IORR_BASE0),
    MsrVal(0xC0010017, MISCREG_IORR_BASE1),
    MsrVal(0xC0010018, MISCREG_IORR_MASK0),
    MsrVal(0xC0010019, MISCREG_IORR_MASK1),
    MsrVal(0xC001001A, MISCREG_TOP_MEM),
    MsrVal(0xC001001D, MISCREG_TOP_MEM2),
    MsrVal(0xC0010114, MISCREG_VM_CR),
    MsrVal(0xC0010115, MISCREG_IGNNE),
    MsrVal(0xC0010116, MISCREG_SMM_CTL),
    MsrVal(0xC0010117, MISCREG_VM_HSAVE_PA)
};

static const unsigned msrMapSize = sizeof(msrMapData) / sizeof(msrMapData[0]);

const MsrMap msrMap(msrMapData, msrMapData + msrMapSize);

bool
msrAddrToIndex(MiscRegIndex &regNum, Addr addr)
{
    MsrMap::const_iterator it(msrMap.find(addr));
    if (it == msrMap.end()) {
        return false;
    } else {
        regNum = it->second;
        return true;
    }
}

} // namespace X86ISA
