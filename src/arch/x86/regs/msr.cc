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
 */

#include "arch/x86/regs/msr.hh"

namespace gem5
{

namespace X86ISA
{

typedef MsrMap::value_type MsrVal;

const MsrMap::value_type msrMapData[] = {
    MsrVal(0x10, misc_reg::Tsc),
    MsrVal(0x1B, misc_reg::ApicBase),
    MsrVal(0xFE, misc_reg::Mtrrcap),
    MsrVal(0x174, misc_reg::SysenterCs),
    MsrVal(0x175, misc_reg::SysenterEsp),
    MsrVal(0x176, misc_reg::SysenterEip),
    MsrVal(0x179, misc_reg::McgCap),
    MsrVal(0x17A, misc_reg::McgStatus),
    MsrVal(0x17B, misc_reg::McgCtl),
    MsrVal(0x1D9, misc_reg::DebugCtlMsr),
    MsrVal(0x1DB, misc_reg::LastBranchFromIp),
    MsrVal(0x1DC, misc_reg::LastBranchToIp),
    MsrVal(0x1DD, misc_reg::LastExceptionFromIp),
    MsrVal(0x1DE, misc_reg::LastExceptionToIp),
    MsrVal(0x200, misc_reg::MtrrPhysBase0),
    MsrVal(0x201, misc_reg::MtrrPhysMask0),
    MsrVal(0x202, misc_reg::MtrrPhysBase1),
    MsrVal(0x203, misc_reg::MtrrPhysMask1),
    MsrVal(0x204, misc_reg::MtrrPhysBase2),
    MsrVal(0x205, misc_reg::MtrrPhysMask2),
    MsrVal(0x206, misc_reg::MtrrPhysBase3),
    MsrVal(0x207, misc_reg::MtrrPhysMask3),
    MsrVal(0x208, misc_reg::MtrrPhysBase4),
    MsrVal(0x209, misc_reg::MtrrPhysMask4),
    MsrVal(0x20A, misc_reg::MtrrPhysBase5),
    MsrVal(0x20B, misc_reg::MtrrPhysMask5),
    MsrVal(0x20C, misc_reg::MtrrPhysBase6),
    MsrVal(0x20D, misc_reg::MtrrPhysMask6),
    MsrVal(0x20E, misc_reg::MtrrPhysBase7),
    MsrVal(0x20F, misc_reg::MtrrPhysMask7),
    MsrVal(0x250, misc_reg::MtrrFix64k00000),
    MsrVal(0x258, misc_reg::MtrrFix16k80000),
    MsrVal(0x259, misc_reg::MtrrFix16kA0000),
    MsrVal(0x268, misc_reg::MtrrFix4kC0000),
    MsrVal(0x269, misc_reg::MtrrFix4kC8000),
    MsrVal(0x26A, misc_reg::MtrrFix4kD0000),
    MsrVal(0x26B, misc_reg::MtrrFix4kD8000),
    MsrVal(0x26C, misc_reg::MtrrFix4kE0000),
    MsrVal(0x26D, misc_reg::MtrrFix4kE8000),
    MsrVal(0x26E, misc_reg::MtrrFix4kF0000),
    MsrVal(0x26F, misc_reg::MtrrFix4kF8000),
    MsrVal(0x277, misc_reg::Pat),
    MsrVal(0x2FF, misc_reg::DefType),
    MsrVal(0x400, misc_reg::Mc0Ctl),
    MsrVal(0x404, misc_reg::Mc1Ctl),
    MsrVal(0x408, misc_reg::Mc2Ctl),
    MsrVal(0x40C, misc_reg::Mc3Ctl),
    MsrVal(0x410, misc_reg::Mc4Ctl),
    MsrVal(0x414, misc_reg::Mc5Ctl),
    MsrVal(0x418, misc_reg::Mc6Ctl),
    MsrVal(0x41C, misc_reg::Mc7Ctl),
    MsrVal(0x401, misc_reg::Mc0Status),
    MsrVal(0x405, misc_reg::Mc1Status),
    MsrVal(0x409, misc_reg::Mc2Status),
    MsrVal(0x40D, misc_reg::Mc3Status),
    MsrVal(0x411, misc_reg::Mc4Status),
    MsrVal(0x415, misc_reg::Mc5Status),
    MsrVal(0x419, misc_reg::Mc6Status),
    MsrVal(0x41D, misc_reg::Mc7Status),
    MsrVal(0x402, misc_reg::Mc0Addr),
    MsrVal(0x406, misc_reg::Mc1Addr),
    MsrVal(0x40A, misc_reg::Mc2Addr),
    MsrVal(0x40E, misc_reg::Mc3Addr),
    MsrVal(0x412, misc_reg::Mc4Addr),
    MsrVal(0x416, misc_reg::Mc5Addr),
    MsrVal(0x41A, misc_reg::Mc6Addr),
    MsrVal(0x41E, misc_reg::Mc7Addr),
    MsrVal(0x403, misc_reg::Mc0Misc),
    MsrVal(0x407, misc_reg::Mc1Misc),
    MsrVal(0x40B, misc_reg::Mc2Misc),
    MsrVal(0x40F, misc_reg::Mc3Misc),
    MsrVal(0x413, misc_reg::Mc4Misc),
    MsrVal(0x417, misc_reg::Mc5Misc),
    MsrVal(0x41B, misc_reg::Mc6Misc),
    MsrVal(0x41F, misc_reg::Mc7Misc),
    MsrVal(0xC0000080, misc_reg::Efer),
    MsrVal(0xC0000081, misc_reg::Star),
    MsrVal(0xC0000082, misc_reg::Lstar),
    MsrVal(0xC0000083, misc_reg::Cstar),
    MsrVal(0xC0000084, misc_reg::SfMask),
    MsrVal(0xC0000100, misc_reg::FsBase),
    MsrVal(0xC0000101, misc_reg::GsBase),
    MsrVal(0xC0000102, misc_reg::KernelGsBase),
    MsrVal(0xC0000103, misc_reg::TscAux),
    MsrVal(0xC0010000, misc_reg::PerfEvtSel0),
    MsrVal(0xC0010001, misc_reg::PerfEvtSel1),
    MsrVal(0xC0010002, misc_reg::PerfEvtSel2),
    MsrVal(0xC0010003, misc_reg::PerfEvtSel3),
    MsrVal(0xC0010004, misc_reg::PerfEvtCtr0),
    MsrVal(0xC0010005, misc_reg::PerfEvtCtr1),
    MsrVal(0xC0010006, misc_reg::PerfEvtCtr2),
    MsrVal(0xC0010007, misc_reg::PerfEvtCtr3),
    MsrVal(0xC0010010, misc_reg::Syscfg),
    MsrVal(0xC0010016, misc_reg::IorrBase0),
    MsrVal(0xC0010017, misc_reg::IorrBase1),
    MsrVal(0xC0010018, misc_reg::IorrMask0),
    MsrVal(0xC0010019, misc_reg::IorrMask1),
    MsrVal(0xC001001A, misc_reg::TopMem),
    MsrVal(0xC001001D, misc_reg::TopMem2),
    MsrVal(0xC0010114, misc_reg::VmCr),
    MsrVal(0xC0010115, misc_reg::Ignne),
    MsrVal(0xC0010116, misc_reg::SmmCtl),
    MsrVal(0xC0010117, misc_reg::VmHsavePa)
};

static const unsigned msrMapSize = sizeof(msrMapData) / sizeof(msrMapData[0]);

const MsrMap msrMap(msrMapData, msrMapData + msrMapSize);

bool
msrAddrToIndex(RegIndex &reg_num, Addr addr)
{
    auto it = msrMap.find(addr);
    if (it == msrMap.end()) {
        return false;
    } else {
        reg_num = it->second;
        return true;
    }
}

} // namespace X86ISA
} // namespace gem5
